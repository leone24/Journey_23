'''
main page:
choose an operation:
1- view contacts
2- edit contact
3- add contact
4- delete contact
5- exit

add contact:
create list
want data from user
--> name-str
    surname-str
    number-int
    add it into list

view contacts:
open list and print one by one
number every contact
want data from user to understand which one user wants to view
acording to the data show name+surname+number

'''
import os

max_id = 0
contacts = []
ROOT_F = ".\contacts"

def main():
   first()
   while True:
    f =  int(input("""choose an operation:
      1- view contacts
      2- edit contact
      3- add contact
      4- erase contact
      5- exit \n """))

    if f == 1:
      print("view contacts")
      viewcon()

    elif f == 2:
      print("edit contacts")
      editcon()

    elif f == 3:
      print("add contact")
      addcon()

    elif f == 4:
      print("delete contact")
      erasecon()

    elif f == 5:
      exit(0)

    else:
      print(" please enter an available operation!")
      continue

def takeinf():
  while True:
    name = str(input("Name: "))
    sname = str (input("Surname: "))
    number = int(input("Phone Number: "))

    if not name or not sname or not number:
      print ("Please fill all the informations to finish the process succesfully.")
      continue

    if len(str(number)) < 10:
        print("Please enter a valid phone number.")
        continue
    else:
        break
  return {"name": name, "surname": sname, "number": number}

def addcon():
  global max_id
  contact = takeinf()
  ID = max_id +1
  max_id = ID
  contact [" id" ]= ID
  with open(f"{ROOT_F}\{ID}", "w") as f:
    f.write(contact["name"] + "\n")
    f.write(contact["surname"] + "\n")
    f.write(contact["number"] + "\n")
    contacts.append(contact)
    print ("Contact saved succesfully.")

def first():
  global max_id

  try:
    for i in os.listdir(ROOT_F):
      with open (f"{ROOT_F}\{i}", "r") as f:
        name, sname, number = f.readlines
        contacts.append ( {"name" : name.strip(), "surname" : sname.strip(), "number" : number.strip(), "ID" : int(i) }); 
        if int(i) > max_id:
            int(i) = max_id
  except ValueError:
    print("Corrupt database files...")
    exit(1)

  except FileNotFoundError:
    os.mkdir(ROOT_F)
    
def viewcon():
    pass

def editcon():
  pass

def erasecon():
  pass

main()
