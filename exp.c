#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <time.h>

int main(int argc, char *argv[]) {
  
 struct timespec mytimespec;
 mytimespec.tv_sec = 0;
 mytimespec.tv_nsec = 5000000;  //5ms 

 void *mem;
 static size_t count=0;
 size_t read_array[1500];
 printf("Shared Memory \n");
   
 off_t offset = 0x3d800000;   //Starting address from 728MB RAM
 off_t len = 0x28000000;       // 40MB for shared memory communication
 off_t write_offset = 0x100000;   //1 MB write offset
  
    // Truncate offset to a multiple of the page size, or mmap will fail.
    size_t pagesize = sysconf(_SC_PAGE_SIZE);
    off_t page_base = (offset / pagesize) * pagesize;
    off_t page_offset = offset - page_base;

   
    int fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (fd == -1) {
        printf("Can't open /dev/mem\n");
        exit(0);
    }
    printf("/dev/mem opened.\n");

    mem = mmap(NULL, page_offset + len, PROT_READ | PROT_WRITE, MAP_SHARED, fd, page_base);
    if (mem == MAP_FAILED) {
        perror("Can't map memory");
        return -1;
    }

printf("Memory mapped at address %p.\n", mem); 

clock_t start = clock();
clock_t start1 = clock();

*((volatile unsigned char *) (mem + page_offset+1)) = 1;  // write flag is initialized to 1

size_t i,j;
while(1)
{
     if (*((volatile unsigned char *) (mem + page_offset)) == 0 ) // assert read flag is equal to 0
      {
            for (i = 4; i < 1504; ++i)
            {
             read_array[i-4] = *((volatile unsigned char *) (mem + page_offset+i));
           // printf("%02x ", *((volatile unsigned char *) (mem + page_offset+i)));
            }

            printf("Read-Success \n");
            
           *((volatile unsigned char *) (mem + page_offset)) = 1; // set read flag to 1   
            printf ( "%f\n", ( (double)clock() - start ) / CLOCKS_PER_SEC );  
            start1 = clock();
            
       }
      else
      {
         printf("wait1 \n");
        //printf("offset: %02x ", *((volatile unsigned char *) (mem + page_offset)));
      }


     if (*((volatile unsigned char *) (mem + page_offset+1)) == 1 ) //assert write flag is equal to 1
      {
         
            for (j = 0; j < 1504; ++j)
            {
             *((volatile unsigned char *) (mem + page_offset+write_offset+j)) = count;   
            }    

            if (count == 100)
               {
                count=0;
               }
                else
              {
               count++;
               }   
                 printf("write -Success \n");
            *((volatile unsigned char *) (mem + page_offset+1)) = 0; // set write flag to 0
            printf ( "%f\n", ( (double)clock() - start1 ) / CLOCKS_PER_SEC );  
            
 
       }
      else
      {
        printf("wait2 \n");
        //printf("offset: %02x ", *((volatile unsigned char *) (mem + page_offset)));
      }

nanosleep(&mytimespec,NULL);
start = clock();

}

if (munmap(mem, page_offset + len) == -1) {
       printf("Can't unmap memory from user space.\n");
      exit(0);
   }
 
close(fd);
    
return 0;
}
