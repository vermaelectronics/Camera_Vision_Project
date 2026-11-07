#!/usr/bin/env python3
# Regenerates sfap_tables.h (twiddles and window) for SFAP_N = 64.
import math, os
N = 64
here = os.path.dirname(os.path.abspath(__file__))
tw = [(round(65536 * math.cos(2 * math.pi * n / N)), round(-65536 * math.sin(2 * math.pi * n / N))) for n in range(N // 2)]
win = [round(65536 * math.sin(math.pi * n / N)) for n in range(N)]
with open(os.path.join(here, 'sfap_tables.h')) as f:
    head = f.read().split('#ifndef SFAP_TABLES_H')[0]
with open(os.path.join(here, 'sfap_tables.h'), 'w') as f:
    f.write(head + '#ifndef SFAP_TABLES_H\n#define SFAP_TABLES_H\n')
    f.write('static const int SFAP_TW_RE[%d] = {%s};\n' % (N // 2, ', '.join(str(a) for a, b in tw)))
    f.write('static const int SFAP_TW_IM[%d] = {%s};\n' % (N // 2, ', '.join(str(b) for a, b in tw)))
    f.write('static const int SFAP_WIN[%d] = {%s};\n' % (N, ', '.join(str(w) for w in win)))
    f.write('#endif\n')
