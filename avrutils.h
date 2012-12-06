#define readCNT1(X)         X = TCNT1L; \
        					X += TCNT1H << 8;
