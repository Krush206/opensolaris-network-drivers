#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>

int
main(int argc, char **argv)
{
	int	c;
	int	i;

	i = 0;
	while (1) {
		i++;
		c = getchar();
		if (c == EOF) {
			break;
		}
		printf("0x%02x, ", c);
		if (i % 8 == 0) {
			printf("\n");
#if 0
			printf("/* %x */\t", i);
#endif
		}
	}
	if (i % 8 != 1) {
		printf("\n");
	}
	exit(0);
}
