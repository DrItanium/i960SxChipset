I960BASE=~/i960-CTOOLS-with-NINDY/i386-nbsd1-ctools/ ic960 -c -R -O2 -ACA branch.c
I960BASE=~/i960-CTOOLS-with-NINDY/i386-nbsd1-ctools/ ic960 -c -R -O2 -ACA brk.c
I960BASE=~/i960-CTOOLS-with-NINDY/i386-nbsd1-ctools/ ic960 -c -R -O2 -ACA copyrght.c
I960BASE=~/i960-CTOOLS-with-NINDY/i386-nbsd1-ctools/ ic960 -c -R -O2 -ACA dto_stub.c
I960BASE=~/i960-CTOOLS-with-NINDY/i386-nbsd1-ctools/ ic960 -c -R -O2 -ACA errno.c
I960BASE=~/i960-CTOOLS-with-NINDY/i386-nbsd1-ctools/ ic960 -c -R -O2 -ACA -I../include fileio.c
I960BASE=~/i960-CTOOLS-with-NINDY/i386-nbsd1-ctools/ ic960 -c -R -O2 -ACA gcc_stub.c
I960BASE=~/i960-CTOOLS-with-NINDY/i386-nbsd1-ctools/ ic960 -c -R -O2 -ACA newlib.c
I960BASE=~/i960-CTOOLS-with-NINDY/i386-nbsd1-ctools/ ic960 -c -R -O2 -ACA profile.c
I960BASE=~/i960-CTOOLS-with-NINDY/i386-nbsd1-ctools/ ic960 -c -R -O2 -ACA ver960.c
I960BASE=~/i960-CTOOLS-with-NINDY/i386-nbsd1-ctools/ asm960 -ACA -o br_asm.o br_asm.s
I960BASE=~/i960-CTOOLS-with-NINDY/i386-nbsd1-ctools/ asm960 -ACA -o heapsize.o heapsize.s
I960BASE=~/i960-CTOOLS-with-NINDY/i386-nbsd1-ctools/ asm960 -ACA -o libasm.o libasm.s
I960BASE=~/i960-CTOOLS-with-NINDY/i386-nbsd1-ctools/ asm960 -ACA -o pr_start.o pr_start.s
I960BASE=~/i960-CTOOLS-with-NINDY/i386-nbsd1-ctools/ asm960 -ACA -o pr_freq.o pr_freq.s
I960BASE=~/i960-CTOOLS-with-NINDY/i386-nbsd1-ctools/ asm960 -ACA -o pr_end.o pr_end.s
I960BASE=~/i960-CTOOLS-with-NINDY/i386-nbsd1-ctools/ asm960 -ACA -o pr_buck.o pr_buck.s
I960BASE=~/i960-CTOOLS-with-NINDY/i386-nbsd1-ctools/ arc960 -sr libnin.a copyrght.o br_asm.o branch.o brk.o dto_stub.o errno.o fileio.o gcc_stub.o heapsize.o libasm.o newlib.o 
I960BASE=~/i960-CTOOLS-with-NINDY/i386-nbsd1-ctools/ arc960 -sr libnin.a pr_start.o pr_freq.o pr_end.o pr_buck.o profile.o ver960.o
cd ..