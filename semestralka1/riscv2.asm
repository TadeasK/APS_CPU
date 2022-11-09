addi a0, x0, 10
addi a1, x0, 2
addi a2, x0, 5

add t1, a0, a1 # 12
and t2, a0, a1 # 2
sub t3, a0, a2 # 5
slt t4, a1, a2 # 1
div t5, a0, a2 # 2
rem t6, a2, a1 # 1

endless:
j endless