SECTIONS
{
    _ram_start = ORIGIN(RAM);
    _ram_size = LENGTH(RAM);

    /* Set stack top to end of RAM; stack limit is bottom of stack */
    __StackTop = ORIGIN(RAM) + LENGTH(RAM);
    __StackLimit = __StackTop - _stack_size;
    PROVIDE(__stack = __StackTop);

        /* Top of head is the bottom of the stack, without the DMZ */
    __DmzBase = __StackLimit - _dmz_size;
    __HeapLimit = __DmzBase;
}
