// Lab 3 template file
.data

.global login_name
login_name:
    .asciz "doshi36"

.global puid
puid:
    .asciz "0033436572"

.balign 4

.equ TEST_SORT,         0x1
.equ TEST_STRCONCAT,    0x2
.equ TEST_FIB,          0x4
.equ TEST_BSEARCH,      0x8
.equ TEST_ALL,          (TEST_SORT | TEST_STRCONCAT | TEST_FIB | TEST_BSEARCH)

// Replace the TEST_ALL with the tests you want to run
// OR expression also support like the TEST_ALL above
// i.e. replace the TEST_ALL below with TEST_SORT | TEST_STRCONCAT
//      will test on the sort and strconcat
.global test
test:
    .word TEST_ALL


// Align to 4bytes boundary as we have a string above
.balign 4

.text
.global asm_sort_int
asm_sort_int:
    /* void asm_sort_int(int32_t* arr, uint32_t n)
     * relies on `qsort` in C standard library to 
     * sort in ascending order
     **/

    /* Enter your code after this comment */
    // Suppose we call the above function `int add_one(int a)` with `a = 1`

    addi x12, x0, 4 
    la x13, asm_cmp
    jal x1, qsort

    /* Enter your code above this comment */
    ret

.global asm_cmp
asm_cmp:
    // int asm_cmp(const void * a, const void * b)
    // Compare function to int32_t
    // used for qsort
    /* Enter your code after this comment */

    lw x12, 0(x10)
    lw x13, 0(x11)
    addi x15, x0, 1

    sub x14, x12, x13
    bge x14, 0, target 
    addi x10, x15, -2
    ret

    target:
        add x10, x0, x15 # t0 = t1 + imm
    /* Enter your code above this comment */ 
    ret

.global asm_strconcat
asm_strconcat:
    /* char* asm_strconcat(char * str1, char * str2)
     * First assign memory space with `malloc`, 
     * concatenating `str1` and `str2`, 
     * and return the resulted string.
     **/
    /* Enter your code after this comment */

    //str1 has been saved in n1 
    addi sp, sp, -8
    sw x10, 0(sp)
    sw x11, 4(sp)
    jal x1, strlen

    sw x12, 0(x10) //x12 -> n1

    lw x10, 0(sp)  
    lw x11, 4(sp)  
    addi sp, sp, 8

    //str2 has been saved in n2
    addi sp, sp, -12
    sw x10, 0(sp)
    sw x11, 4(sp)
    sw x12, 8(sp)
    add x10, x0, x11
    jal x1, strlen

    sw x13, 0(x10) //x13 -> n2

    lw x10, 0(sp)  
    lw x11, 4(sp)  
    lw x12, 8(sp)
    addi sp, sp, 12

    addi x14, x12, 1
    add x14, x14, x13 //size = n1 + n2 + 1 

    addi sp, sp, -20
    sw x10, 0(sp)
    sw x11, 4(sp)
    sw x12, 8(sp)
    sw x13, 12(sp)
    sw x14, 16(sp)

    add x10, x0, x14
    jal x1, malloc

    sw x15, 0(x10) // buf = return value of malloc

    lw x10, 0(sp)  
    lw x11, 4(sp)  
    lw x12, 8(sp)
    lw x13, 12(sp)
    lw x14, 16(sp)    
    addi sp, sp, 20

    //memcpy(buf, str1, n1)
    addi sp, sp, -24
    sw x10, 0(sp)
    sw x11, 4(sp)
    sw x12, 8(sp)
    sw x13, 12(sp)
    sw x14, 16(sp)
    sw x15, 20(sp)

    add x11, x10, x0
    add x10, x15, x0
    jal x1, memcpy

    lw x10, 0(sp)  
    lw x11, 4(sp)  
    lw x12, 8(sp)
    lw x13, 12(sp)
    lw x14, 16(sp) 
    lw x15, 20(sp)   
    addi sp, sp, 24

    //memcpy(buf + n1, str2, n2)
    addi sp, sp, -24
    sw x10, 0(sp)
    sw x11, 4(sp)
    sw x12, 8(sp)
    sw x13, 12(sp)
    sw x14, 16(sp)
    sw x15, 20(sp)

    add x10, x12, x15
    add x12, x13, x0
    jal x1, memcpy

    lw x10, 0(sp)  
    lw x11, 4(sp)  
    lw x12, 8(sp)
    lw x13, 12(sp)
    lw x14, 16(sp) 
    lw x15, 20(sp)   
    addi sp, sp, 24

    addi x14, x14, -1
    add x15, x14, x15
    sb x15, 0(x0) 
    sub x15, x15, x14 
    
    sw x15, 0(x10)
    /* Enter your code above this comment */
    ret 

.global asm_fib
asm_fib:
    /* uint32_t asm_fib(uint32_t n)
     * accept an index term `n` and return 
     * the Fn fibonacci term (F0 = 0, F1 = 1)
     **/

    /* Enter your code after this comment */
    add x14, x0, x0    
    addi x14, x14, 2
    blt x10, x14, default 
    
    //asm_fib(n - 1)
    addi sp, sp, -8
    sw x1, 0(sp)
    sw x10, 4(sp)

    addi x10, x10, -1
    jal x1, asm_fib

    sw x11, 0(x10)

    lw x1, 0(sp)
    lw x10, 4(sp)
    addi sp, sp, 8

    //asm_fib(n - 2)
    addi sp, sp, -12
    sw x1, 0(sp)
    sw x10, 4(sp)
    sw x11, 8(sp)
    
    addi x10, x10, -2
    jal x1, asm_fib

    sw x12, 0(x10)

    lw x1, 0(sp)
    lw x10, 4(sp)
    lw x11, 8(sp)
    addi sp, sp, 12

    add x10, x11, x12

    default:
    /* Enter your code above this comment */
    ret

.global asm_bsearch
asm_bsearch:
    /* int32_t asm_bsearch(int *arr, int key, 
                    int32_t start,
                    int32_t end)
     * Return the index of the element in the array
     * if the element does not exist, return -1 instead
     * arr: integer array to be searched
     * key: the element we want to search on
     * start: start index of subarray, inclusive
     * end: end index of subarray, inclusiveloop through an entire string and only convert
     **/

    /* Enter your code after this comment */
    addi x15, x0, -1
    bgt x11, x12, target # if x11 >x121 then target
    
    # uint32_t mid = (end + start) / 2;
    add x16, x11, x12
    srl x16, x16, 1

    add x10, x10, x16 
    sw x14, 0(x10)
    sub x10, x10, x16 

    bge x14, x11, else_if_1 # if t0 >= t1 then target
    
    addi sp, sp, -24
    sw x10, 0(sp)
    sw x11, 4(sp)
    sw x12, 8(sp)
    sw x13, 12(sp)
    sw x16, 16(sp) 
    sw x1, 20(sp)

    addi x12, x16, 1
    jal x1, asm_bsearch

    sw x17, 0(x10)

    lw x10, 0(sp)
    lw x11, 4(sp)
    lw x12, 8(sp)
    lw x13, 12(sp)
    lw x16, 16(sp) 
    lw x1, 20(sp)
    addi sp, sp, 24

    sw x17, 0(x10)
    ret

    else_if_1:
        bge x11, x14, else_if_2 
        addi sp, sp, -24
        sw x10, 0(sp)
        sw x11, 4(sp)
        sw x12, 8(sp)
        sw x13, 12(sp)
        sw x16, 16(sp) 
        sw x1, 20(sp)

        addi x13, x16, -1
        jal x1, asm_bsearch

        sw x17, 0(x10)

        lw x10, 0(sp)
        lw x11, 4(sp)
        lw x12, 8(sp)
        lw x13, 12(sp)
        lw x16, 16(sp) 
        lw x1, 20(sp)
        addi sp, sp, 24

        sw x17, 0(x10)
        ret

    else_if_2:
        add x10, x0, x16
    
    /* Enter your code above this comment */
    target:
        add x10, x10, x15
    ret
