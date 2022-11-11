addi a1, zero, 8
lw a1, 0(a1)
addi a2, x0, 0


testArithmetic:
addi a0, x0, 0
lw a1, 0(x0)
lw a2, 4(x0)
add a3, a1, a2
call save
sub a3, a1, a2
call save
and a3, a1, a2
call save
div a3, a1, a2
call save
rem a3, a1, a2
call save


testBigNumbers:
    addi a2, x0, 2
    lui a3, 12
    call save
    sll a3, a3, a2
    call save
    addi a2, a2, 12
    srl a3, a3, a2
    call save
    addi a1, x0, 12
    addi a2, x0, 2
    addi a3, x0, 0
    sub a3, a3, a1
    srl a3, a3, a2
    call save
    sll a3, a3, a2
    call save
    sra a3, a3, a2
    call save
    


end:
    j end

save:
    sw a3, 0(a0)
    addi a0, a0, 4
    ret
