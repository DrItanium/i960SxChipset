/* 
 * TODO: adapt for my purposes 
 *
 */

MEMORY
{
    rom (rx)  : ORIGIN = 0, l = 8M
    ram (!rx) : o = 0x80000000, l = 16M
}

SECTIONS
{
    .text :
    {

    } >rom
    .data :
    {

    } >ram
    .bss :
    {

    } >ram
}

cs1 = -(system_address_table + prcb_ptr + start_ip) ;

/* STARTUP ( CMakeFiles/hitagimon.dir/sx_init.s.o ) */
