/* J J Larkworthy 25 Aug 2006
 * Program to set the boot rom parameters to load the stage 1 loader
 *
 * build this program using the following command line:

gcc installer.c -o installer

 * use the program on a copy of the boot sector.
 *
dd if=/dev/sda of=temp count=1 bs=512
./installer temp
dd if=temp of=/dev/sda count=1 bs=512

 *
 */

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>


#define SECTOR_START 1
#define SECTOR_LENGTH 0x7C00

void usage(void)
{
    printf("installer <file>\n");
    printf
	(" - installer for the boot rom parameters to load stage1 loader\n");
}

int main(int argc, char **argv)
{

    int status;
    unsigned long data_value;
    int in_file;

    if (argc != 2) {
	usage();
	exit(-1);
    }

    in_file = open(argv[1], O_WRONLY);

    lseek(in_file, 0x1b0, SEEK_SET);

    data_value = SECTOR_START + SECTOR_LENGTH;

    write(in_file, &data_value, 4);
    data_value = SECTOR_START;
    write(in_file, &data_value, 4);
    data_value = SECTOR_LENGTH;
    write(in_file, &data_value, 4);
    close(in_file);

}
