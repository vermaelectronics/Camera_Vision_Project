function c = gps_ca_code(prn)
%GPS_CA_CODE 1023-chip GPS C/A Gold code (+1/-1, bit 0 -> +1), IS-GPS-200.
taps = [2 6;3 7;4 8;5 9;1 9;2 10;1 8;2 9;3 10;2 3;3 4;5 6;6 7;7 8;8 9;9 10; ...
        1 4;2 5;3 6;4 7;5 8;6 9;1 3;4 6;5 7;6 8;7 9;8 10;1 6;2 7;3 8;4 9];
t = taps(prn, :);
g1 = ones(1, 10); g2 = ones(1, 10);
c = zeros(1, 1023);
for i = 1:1023
    chip = xor(g1(10), xor(g2(t(1)), g2(t(2))));
    c(i) = 1 - 2 * chip;
    f1 = xor(g1(3), g1(10));
    f2 = mod(g2(2) + g2(3) + g2(6) + g2(8) + g2(9) + g2(10), 2);
    g1 = [f1, g1(1:9)];
    g2 = [f2, g2(1:9)];
end
end
