/* J J Larkworthy  20 March 2006
 * sata.h 
 * external functions defined for loading from a sata disc
 */

/* execute the sata disk loader to obtain an executable image from
 * the first sata disk drive.
 * report the number of sectors read.
 * This should be greater than zero for a valid image. 
 */
extern u32 run_sata(u32 location, u32 length, u32* memory_location, int disk);

