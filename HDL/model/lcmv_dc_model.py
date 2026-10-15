"""Reference model for the 2-antenna robust LCMV core (PIER C vol. 4, 2008).

Checks that the closed form used in lcmv_dc_weight_2ch.v,
    w2 = (1-eps)(s-j)/|s-j|^2,  w1 = 1 - conj(s) w2,
equals the paper's LCMV solution  w = R^-1 C (C^H R^-1 C)^-1 f  for any R,
compares it with the weights produced by the HDL simulation, and shows why
the derivative constraint cannot be used with 2 antennas.

    python3 lcmv_dc_model.py
"""
import numpy as np

rng = np.random.default_rng(0)


def steer(theta_deg):
    return np.array([1, np.exp(1j * np.pi * np.sin(np.deg2rad(theta_deg)))])


def lcmv_paper(r, c, f):
    """Paper eq. (7)/(9): w = R^-1 C (C^H R^-1 C)^-1 f."""
    ri_c = np.linalg.solve(r, c)
    return ri_c @ np.linalg.solve(c.conj().T @ ri_c, f)


def lcmv_closed_form(gps_deg, jam_deg, eps=0.0):
    s, j = steer(gps_deg)[1], steer(jam_deg)[1]
    w2 = (1 - eps) * (s - j) / abs(s - j) ** 2
    return np.array([1 - np.conj(s) * w2, w2])


def random_cov(m=2, k=4096):
    x = rng.standard_normal((m, k)) + 1j * rng.standard_normal((m, k))
    x += np.outer(steer(40), 300 * (rng.standard_normal(k) + 1j * rng.standard_normal(k)))
    return x @ x.conj().T / k


if __name__ == "__main__":
    gps, jam = 10, 40
    for eps in (0.0, 0.001, 0.006):
        c = np.column_stack([steer(gps), steer(jam)])
        f = np.array([1, eps])
        w_p = lcmv_paper(random_cov(), c, f)
        w_c = lcmv_closed_form(gps, jam, eps)
        print(f"eps={eps:<6} paper formula == closed form: {np.allclose(w_p, w_c)}  "
              f"|w^H a_s|={abs(w_c.conj() @ steer(gps)):.4f}  "
              f"|w^H a_J|={abs(w_c.conj() @ steer(jam)):.4f}")

    # HDL simulation result (tb case 1: gps=10, jam=40), Q16.16
    w_hdl = np.array([32768 + 36109j, 46749 - 13865j]) / 65536
    w_ref = lcmv_closed_form(gps, jam)
    print("HDL weights :", np.round(w_hdl, 5))
    print("float model :", np.round(w_ref, 5))
    print(f"max |error| : {np.max(abs(w_hdl - w_ref)):.2e}")

    # Derivative constraint with M = 2: w^H B a_J = conj(w2) e^{j psi} = 0 -> w2 = 0
    b = np.diag([0, 1])
    c3 = np.column_stack([steer(gps), b @ steer(jam), steer(jam)])
    print("rank of [a_s, B a_J, a_J] with M=2:", np.linalg.matrix_rank(c3),
          "(3 constraints, only 2 degrees of freedom -> eq. 9 infeasible)")
