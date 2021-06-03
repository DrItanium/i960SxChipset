/* 
 * TODO: adapt for my purposes 
 *
 */

MEMORY
{
    rom (rx)  : ORIGIN = 0, l = 64M
    ram (!rx) : o = 0x80000000, l = 512M
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
    .heap : {
        __heap_start__ = .;
        end = __heap_start__;
        _end = end;
        __end = end;
        KEEP(*(.heap))
        __heap_end__ = .;
        __HeapLimit = __heap_end__;
    } >ram
}

cs1 = -(system_address_table + prcb_ptr + start_ip) ;

/* STARTUP ( CMakeFiles/hitagimon.dir/sx_init.s.o ) */
/*
How to dump the actual boot code:

i960-elf-objcopy --only-section=.text -O binary hitagimon hitagimon.bin

You have to make sure that you don't dump out the data section because that can get very large!
*/