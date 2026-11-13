#!/usr/bin/env python3
# Regenerates sfap_tables.h (twiddles and window) for SFAP_TMAX = 128 points.
# A 64-bin core uses every second entry (see sfap_pi.cpp: SFAP_TSTEP).
import math, os
NMAX = 128
here = os.path.dirname(os.path.abspath(__file__))
tw = [(round(65536 * math.cos(2 * math.pi * n / NMAX)), round(-65536 * math.sin(2 * math.pi * n / NMAX))) for n in range(NMAX // 2)]
win = [round(65536 * math.sin(math.pi * n / NMAX)) for n in range(NMAX)]
def rows(v):
    return ",\n    ".join(", ".join(str(x) for x in v[i:i + 8]) for i in range(0, len(v), 8))
with open(os.path.join(here, 'sfap_tables.h')) as f:
    head = f.read().split('#ifndef SFAP_TABLES_H')[0]
with open(os.path.join(here, 'sfap_tables.h'), 'w') as f:
    f.write(head + '#ifndef SFAP_TABLES_H\n#define SFAP_TABLES_H\n#define SFAP_TMAX %d\n' % NMAX)
    f.write('static const int SFAP_TW_RE[%d] = {\n    %s};\n' % (NMAX // 2, rows([a for a, b in tw])))
    f.write('static const int SFAP_TW_IM[%d] = {\n    %s};\n' % (NMAX // 2, rows([b for a, b in tw])))
    f.write('static const int SFAP_WIN[%d] = {\n    %s};\n' % (NMAX, rows(win)))
    f.write('#endif\n')
