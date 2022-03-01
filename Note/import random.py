import random
total = 0
for i in range(10000000):
    number1 = random.randint(0,100)
    number2 = random.randint(0,100)
    count=0
    while number1 != number2:
        number2 = random.randint(0,100)
        count+=1
    total+=count

print(total/10000000)