# TODO: adapt for my purposes
MEMORY
{
    rom:   o=0x0, l=0x4000
    ram:   o=0x5000, l=0x35000
}

SECTIONS
{
    .text:
    {

    } >rom
    .data:
    {

    } >ram
    .bss:
    {

    } >ram
}

cs1 = -(system_address_table + prcb_ptr + start_ip);