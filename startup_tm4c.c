/*Startup code for TM4C*/
extern int CSTACK$$Limit;
void __iar_program_start(void);

int const __vector_table[] @ ".intvec"= {
    (int)&CSTACK$$Limit,
    (int)&__iar_program_start //gives adress of func

      
      
};