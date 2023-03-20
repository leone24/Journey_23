#input from user
#introduce possible operatioins
#what is our operation
#show the result
while True:
    num = float(input("Enter first number: "))
    print ('''
        + : addition
        -: subtraction
        *: multiplication
        /: division
        //: floor division
        %: modulus
        **:  exponentiation
        ''')
    a = input("choose an operation :")
    
    num1 = float(input("Enter second number: "))



    if a == "+" :
        print (num, "+" , num1, "=" ,(num + num1))
    
    elif a == "-" :
        print (num, "-" , num1, "=" ,(num - num1))   

    elif a == "*" :
        print (num, "*" , num1, "=" ,(num * num1))

    elif a == "/" :
        print (num, "/" , num1, "=" ,(num / num1))

    elif a == "//" :
        print (num, "//" , num1, "=" ,(num // num1))

    elif a == "%" :
        print (num, "%" , num1, "=" ,(num % num1))

    elif a == "**" :
        print (num, "**" , num1, "=" ,(num ** num1))

    else:
        print(" Please enter an available symbol!")
    


