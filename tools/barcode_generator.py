from barcode.writer import ImageWriter
import barcode

def generate_barcode(alphanum_string):
    options = {
        'module_width': 0.3,  # Width of each module (bar)
        'module_height': 7.0,  # Height of each module (bar)
        'quiet_zone': 2.0,  # Width of the quiet zone on each side
        'font_size': 5,  # Font size for the text below the barcode (0 to remove text)
        'text_distance': 0.0,  # Distance between the barcode and the text
        'background': 'white',  # Background color
        'foreground': 'black',  # Foreground color
        'write_text': True  # Whether to write the text below the barcode
    }
    
    code128 = barcode.get_barcode_class('code128')
    barcode_instance = code128(alphanum_string, writer=ImageWriter())
    
    return barcode_instance.save(alphanum_string, options=options)

if __name__ == "__main__":
    continuing = True
    temp = ""
    while temp.lower() != "n":
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
        generate_barcode(data)
        print("Barcode code created!")
        
        temp = input("Would you like to create another barcode? (y/n): ")
        while temp.lower() != "n" and temp.lower() != "y":
            print("Input y or n!")
            temp = input("Would you like to create another barcode? (y/n): ")
