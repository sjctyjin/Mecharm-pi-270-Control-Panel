def sum(num=10):
    sum = 0
    for i in range(num):
        sum = sum + (i+1)
    return sum

if __name__ == "__main__":
    total = sum(num=10)
    print(total)