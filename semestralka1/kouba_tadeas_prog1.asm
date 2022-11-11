.data
.align 2

.text

main:
    addi t0, zero, 8
    lw  s0, 0(t0) # Pointer to first item
    addi t0, zero, 4
    lw s1, 0(t0) # Size of array

    add t0, zero, zero # i = 0
for:
    beq t0, s1, done # i ?== size of arr
    lw a0, 0(s0)    # Load item
    jal ra, prime  # Test if item is prime
    sw a1, 0(s0)  # Set array[item] to 1 if it was prime else to 0
    addi s0, s0, 4 # Next item
    addi t0, t0, 1 # i++
    jal zero, for # continue
done:
    jal zero, done # Endless cycle at the end

# prime:
# input a0 - number to test
# output a1 - 1 if number is prime else 0
prime:
    addi t1, zero 2 # i = 2
    blt a0, t1, notPrime # if num < 2 it is not prime

forPrime:
    beq t1, a0, isPrime # If i == num num is prime
    rem t2, a0, t1    # set t2 to num MOD i
    beq t2, zero, notPrime # if t2 MOD i == 0 number is not prime
    addi t1, t1, 1  # i++
    jal zero, forPrime # continue

isPrime:
    addi a1, zero, 1 # return = 1
    jalr zero, ra, 0 # return

notPrime:
    add a1, zero, zero # return = 0
    jalr zero, ra, 0 # return



