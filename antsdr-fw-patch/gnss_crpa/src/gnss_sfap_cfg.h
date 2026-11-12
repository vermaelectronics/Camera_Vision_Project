/* gnss_sfap_cfg.h -- SFAP-PI FFT size (antsdr-fw-patch crpa6).
 * AJ_LOG2N = 7: 128 bins (default).  AJ_LOG2N = 6: 64 bins (SFAP-PI v5.0).
 * Must match SFAP_LOG2N (hls/sfap_pi) and `AJ_LOG2N (gnss_passthrough.v);
 * build.sh sets all three from SFAP_BINS (128 or 64). */
#ifndef GNSS_SFAP_CFG_H_
#define GNSS_SFAP_CFG_H_
#ifndef AJ_LOG2N
#define AJ_LOG2N 7
#endif
#endif
