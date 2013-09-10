
void AppMain(void);

#include "stdio.h"
int main(void){

	#define FILE_SIZE 100
	int n = 0;
	char *name = "/dev/fake_serial_a";
	char str[FILE_SIZE] = "initial value\n";
	FILE *fp;
	fp = fopen(name, "r+");
	if (!fp)
	{
		fprintf(stdout, "debug : file open error!\n");
	}
	//n = fread(str, 1, FILE_SIZE, fp);
	//fprintf(stdout, "debug : %d bytes read!\n", n);
	//fprintf(stdout, "debug : %s\n", str);
	n = fwrite("xxxxx debug : write from semihosting", 1, 36, fp);
	fprintf(stdout, "debug : %d bytes write!\n", n);

	fclose(fp);

	return 0;

 	AppMain();
	return 0;
}
