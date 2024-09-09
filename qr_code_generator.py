import qrcode

if __name__ == "__main__":
    continuing = True
    while continuing:
        length = "a"
        length = input("Input number of stops: ")
        while not length.isnumeric():
            print("Input a numeric value!")
            length = input("Input number of stops: ")
        stop = "a"
        stops_string = ""
        stops_list = []
        while stop.lower() != "quit":
            stop = input("Input a stop or \"quit\" to stop: ")
            while (not stop.isnumeric() or int(stop) in stops_list) and stop.lower() != "quit":
                print("Invalid input!")
                stop = input("Input a stop or \"quit\" to stop: ")
            if stop.lower() != "quit":
                stops_list.append(int(stop))

        stops_list.sort()
        for i in stops_list:
            if i <= int(length):
                stops_string += str(i) + ","
            else:
                print(str(i) + " was invalid and has been omitted")
        stops_string = stops_string[:len(stops_string)-1]          

        data = "L=" + length + ":S=" + stops_string
        img = qrcode.make(data)
        img.save(data+".png")
        print("QR code created!")
        
        temp = input("Would you like to create another QR code? (y/n): ")
        while temp.lower() != "n" and temp.lower() != "y":
            print("Input y or n!")
            temp = input("Would you like to create another QR code? (y/n): ")

        if temp.lower() == "n":
            continuing = False
