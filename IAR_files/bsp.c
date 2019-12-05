/* Board Support Package */

#include "TM4C1294NCPDT.h"

void assert_failed (char const *file, int line){
  /* TBD: damage control*/
  
  NVIC_SystemReset(); /*Reset the system*/
  
  
}
