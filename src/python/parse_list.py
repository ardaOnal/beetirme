import random
from tkinter import *
from tkinter import filedialog
from tkinter import messagebox
from tkinter import scrolledtext
from PIL import Image, ImageTk
import math
from collections import defaultdict

# shelf index:  1  item:  <RigidBody_[float] name='base_link_cracker' index=69 model_instance=51>
# shelf index:  2  item:  <RigidBody_[float] name='base_link_sugar' index=72 model_instance=54>
# shelf index:  3  item:  <RigidBody_[float] name='base_link_soup' index=75 model_instance=57>
# shelf index:  4  item:  <RigidBody_[float] name='base_link_mustard' index=78 model_instance=60>
# shelf index:  5  item:  <RigidBody_[float] name='base_link_gelatin' index=81 model_instance=63>
# shelf index:  6  item:  <RigidBody_[float] name='base_link_meat' index=84 model_instance=66>
# shelf index:  7  item:  <RigidBody_[float] name='base_link_cracker' index=87 model_instance=69>
# shelf index:  8  item:  <RigidBody_[float] name='base_link_sugar' index=90 model_instance=72>
# shelf index:  9  item:  <RigidBody_[float] name='base_link_soup' index=93 model_instance=75>

items_to_shelves = {"sponge": (1), "sugar": (2), "soup": (3), "mustard": (4), "jello": (5), "meat": (6)}

def def_value():
    return 0

def get_item_inventory(plant):
    inventory = defaultdict(lambda: 0)
    
    for bodyIndex in plant.GetFloatingBaseBodies():
        name = plant.get_body(bodyIndex).name()
        if name == "base_link_cracker":
            inventory["cracker"] += 1
        elif name == "base_link_sugar":
            inventory["sugar"] += 1
        elif name == "base_link_soup":
            inventory["soup"] += 1
        elif name == "base_link_mustard":
            inventory["mustard"] += 1
        elif name == "base_link_gelatin":
            inventory["jello"] += 1
        elif name == "base_link_meat":
            inventory["meat"] += 1
        elif name == "plum_link":
            inventory["plum"] += 1
        elif name == "chips_can_link":
            inventory["chips"] += 1
        elif name == "pudding_box_link":
            inventory["pudding"] += 1
        elif name == "link":
            inventory["beer"] += 1
        elif name == "sponge_link":
            inventory["sponge"] += 1
    print(inventory)
    return inventory

def select_items(plant, items_per_shelf=3):
    inventory = get_item_inventory(plant)

    item_names = []
    max_amounts = []

    for name, count in inventory.items():
        item_names.append(name.capitalize()) # item name
        max_amounts.append(count) # max amount available of that item

    lst = []

    def submit():
        nonlocal lst
        lst = []
        for item, var in item_vars.items():
            amount = int(var.get())

            if amount > 0:
                item_name = item.lower()
                shelf_id = items_to_shelves[item_name]
                if type(shelf_id) is not int:
                    shelves = math.ceil(amount / items_per_shelf)
                    if shelves == 1:
                        shelf_id = random.choice(shelf_id)
                        added = [(item_name, amount, shelf_id)]  # item name, amount, shelf_id
                    else:
                        added = []
                        remaining = amount
                        for j in range(shelves):
                            added.append((item_name, items_per_shelf if remaining >= items_per_shelf else remaining, shelf_id[j]))
                            remaining -= items_per_shelf
                else:
                    added = [(item_name, amount, shelf_id)]
                lst += added
        root.quit()

    def exit_program():
        root.quit()

    def validate_spinbox(new_value, max_value):
        if new_value == "":
            return True
        if new_value.isdigit() and int(new_value) <= int(max_value):
            return True
        return False

    root = Tk()
    root.title("Shopping List")
    root.geometry("500x600")  # Set the default window size

    item_vars = {}  # Dictionary to store the item variables

    # Set the font and size for labels and spinners
    label_font = ("Arial Narrow", 18)
    spinner_font = ("Arial Narrow", 16)

    main_frame = Frame(root)
    main_frame.pack(fill=BOTH, expand=True, padx=20, pady=20)

    canvas = Canvas(main_frame)
    canvas.pack(side=LEFT, fill=BOTH, expand=True)

    scrollbar = Scrollbar(main_frame, orient=VERTICAL, command=canvas.yview)
    scrollbar.pack(side=RIGHT, fill=Y)

    canvas.configure(yscrollcommand=scrollbar.set)

    items_frame = Frame(canvas)
    items_frame.pack(anchor=NW, pady=20)

    max_label_width = max([len(f"{item} (stock: {max_amounts[index]})") for index, item in enumerate(item_names)])  # Get the maximum label width

    images = []
    for name in item_names:
        images.append(Image.open(f"src/python/images/{name.lower()}.jpg"))

    resized_images = [
        image.resize((70, 70), Image.Resampling.LANCZOS) for image in images
    ]

    photo_images = [
        ImageTk.PhotoImage(resized_image) for resized_image in resized_images
    ]

    for i, (item, max_amount) in enumerate(zip(item_names, max_amounts)):
        var = StringVar()
        item_vars[item] = var

        item_frame = Frame(items_frame)
        item_frame.pack(pady=5)

        image_label = Label(item_frame, image=photo_images[i])
        image_label.pack(side=LEFT, padx=5)

        label = Label(item_frame, text=f"{item} (stock: {max_amount})", font=label_font, width=max_label_width, anchor="w")
        label.pack(side=LEFT)

        spinner = Spinbox(item_frame, from_=0, to=max_amount, font=spinner_font, width=5, textvariable=var,
                          validate='key', validatecommand=(item_frame.register(validate_spinbox), '%P', max_amount))
        spinner.pack(side=LEFT)

    canvas.create_window((0, 0), window=items_frame, anchor=NW)

    def on_configure(event):
        canvas.configure(scrollregion=canvas.bbox("all"))

        # Adjust the window size based on the number of items
        item_count = len(item_names)
        if item_count < 10:  # Adjust the number as desired
            root.geometry(f"500x{item_count * 80 + 150}")  # Adjust the height based on the item count

        # Show/hide the scrollbar based on the content height
        content_height = items_frame.winfo_height()
        canvas_height = canvas.winfo_height()
        if content_height <= canvas_height:
            scrollbar.pack_forget()
        else:
            scrollbar.pack(side=RIGHT, fill=Y)

    canvas.bind("<Configure>", on_configure)

    button_frame = Frame(root)
    button_frame.pack(side=BOTTOM, pady=20)

    submit_button = Button(button_frame, text="Submit", command=submit, font=label_font)
    submit_button.pack(side=LEFT, padx=10)

    exit_button = Button(button_frame, text="Exit", command=exit_program, font=label_font)
    exit_button.pack(side=LEFT, padx=10)

    root.mainloop()

    return lst


# Example usage:
inventory = [("Cracker", 6), ("Sugar", 3), ("Soup", 3), ("Mustard", 3), ("Jello", 3), ("Meat", 3)]

# Call the select_items function with the item names and maximum amounts
#print(select_items(inventory))




































































# def select_items(item_names, max_amounts):
#     lst = []

#     def submit():
#         nonlocal lst
#         lst = []
#         for item, var in item_vars.items():
#             amount = int(var.get())

#             if amount > 0:
#                 item_name = item.lower()
#                 shelf_id = items_to_shelves[item_name]
#                 if type(shelf_id) is not int:
#                     shelf_id = random.choice(shelf_id)
#                 item = (item_name, amount, shelf_id) # item name, amount, shelf_id
#                 lst.append(item)

#     def exit_program():
#         root.quit()

#     def validate_spinbox(new_value, max_value):
#         if new_value == "":
#             return True
#         if new_value.isdigit() and int(new_value) <= int(max_value):
#             return True
#         return False

#     root = Tk()
#     root.title("Shopping List")
#     root.geometry("500x600")  # Set the default window size

#     item_vars = {}  # Dictionary to store the item variables

#     # Set the font and size for labels and spinners
#     label_font = ("Arial Narrow", 18)
#     spinner_font = ("Arial Narrow", 16)

#     main_frame = Frame(root)
#     main_frame.pack(fill=BOTH, expand=True, padx=20, pady=20)

#     canvas = Canvas(main_frame)
#     canvas.pack(side=LEFT, fill=BOTH, expand=True)

#     scrollbar = Scrollbar(main_frame, orient=VERTICAL, command=canvas.yview)
#     scrollbar.pack(side=RIGHT, fill=Y)

#     canvas.configure(yscrollcommand=scrollbar.set)

#     items_frame = Frame(canvas)
#     items_frame.pack(anchor=CENTER, pady=20)

#     max_label_width = max([len(item) for item in item_names])  # Get the maximum label width

#     for item, max_amount in zip(item_names, max_amounts):
#         var = StringVar()
#         item_vars[item] = var

#         item_frame = Frame(items_frame)
#         item_frame.pack(pady=5)

#         label = Label(item_frame, text=item, font=label_font, width=max_label_width, anchor=CENTER)
#         label.pack(side=LEFT, padx=5)

#         validate_max_amount = (item_frame.register(validate_spinbox), '%P', max_amount)
#         spinner = Spinbox(item_frame, from_=0, to=max_amount, font=spinner_font, width=7, textvariable=var,
#                           validate='key', validatecommand=validate_max_amount)
#         spinner.pack(side=LEFT, padx=5)

#     canvas.create_window((0, 0), window=items_frame, anchor=NW)

#     def on_configure(event):
#         canvas.configure(scrollregion=canvas.bbox("all"))

#         # Adjust the window size based on the number of items
#         item_count = len(item_names)
#         if item_count < 10:  # Adjust the number as desired
#             root.geometry(f"400x{item_count * 80}")  # Adjust the height based on the item count

#         # Show/hide the scrollbar based on the content height
#         content_height = items_frame.winfo_height()
#         canvas_height = canvas.winfo_height()
#         if content_height <= canvas_height:
#             scrollbar.pack_forget()
#         else:
#             scrollbar.pack(side=RIGHT, fill=Y)

#     canvas.bind("<Configure>", on_configure)

#     def on_mousewheel(event):
#         canvas.yview_scroll(int(-1 * (event.delta / 120)), "units")

#     canvas.bind_all("<MouseWheel>", on_mousewheel)

#     button_frame = Frame(root)
#     button_frame.pack(side=BOTTOM, pady=20)

#     submit_button = Button(button_frame, text="Submit", command=submit, font=label_font)
#     submit_button.pack(side=LEFT, padx=10)

#     exit_button = Button(button_frame, text="Exit", command=exit_program, font=label_font)
#     exit_button.pack(side=LEFT, padx=10)

#     root.mainloop()

#     return lst

# def validate_spinbox(new_value, max_value):
#     if new_value == "":
#         return True
#     if new_value.isdigit() and int(new_value) <= int(max_value):
#         return True
#     return False


# # Example usage:
# item_names = ["Cracker", "Sugar", "Soup", "Mustard", "Jello", "Meat"]
# max_amounts = [10, 5, 8, 3, 6, 2]

# # Call the select_items function with the item names and maximum amounts
# print(select_items(item_names, max_amounts))






























# def select_items(item_names, max_amounts):
#     lst = []
#     def submit():
#         for item, var in item_vars.items():
#             amount = int(var.get())
#             # print(item, amount)  # Print the item name and spinner value

#             if amount != 0:
#                 item_name = item.lower()
#                 shelf_id = items_to_shelves[item_name]
#                 if type(shelf_id) is not int:
#                     shelf_id = random.choice(shelf_id)
#                 item = (item_name, amount, shelf_id) # item name, amount, shelf_id
#                 lst.append(item)
#         root.quit()
    
#     root = Tk()
#     root.title("Shopping List")
#     root.geometry("500x600")  # Set the default window size

#     item_vars = {}  # Dictionary to store the item variables

#     # Set the font and size for labels and spinners
#     label_font = ("Arial Narrow", 18)
#     spinner_font = ("Arial Narrow", 16)

#     main_frame = Frame(root)
#     main_frame.pack(fill=BOTH, expand=True, padx=20, pady=20)

#     canvas = Canvas(main_frame)
#     canvas.pack(side=LEFT, fill=BOTH, expand=True)

#     scrollbar = Scrollbar(main_frame, orient=VERTICAL, command=canvas.yview)
#     scrollbar.pack(side=RIGHT, fill=Y)

#     canvas.configure(yscrollcommand=scrollbar.set)

#     items_frame = Frame(canvas)
#     items_frame.pack(anchor=CENTER, pady=20)

#     max_label_width = max([len(item) for item in item_names])  # Get the maximum label width

#     for item, max_amount in zip(item_names, max_amounts):
#         var = StringVar()
#         item_vars[item] = var

#         item_frame = Frame(items_frame)
#         item_frame.pack(pady=5)

#         label = Label(item_frame, text=item, font=label_font, width=max_label_width, anchor=CENTER)
#         label.pack(side=LEFT, padx=5)

#         validate_max_amount = (item_frame.register(validate_spinbox), '%P', max_amount)
#         spinner = Spinbox(item_frame, from_=0, to=max_amount, font=spinner_font, width=7, textvariable=var,
#                           validate='key', validatecommand=validate_max_amount)
#         spinner.pack(side=LEFT, padx=5)

#     canvas.create_window((0, 0), window=items_frame, anchor=NW)

#     def on_configure(event):
#         canvas.configure(scrollregion=canvas.bbox("all"))

#         # Adjust the window size based on the number of items
#         item_count = len(item_names)
#         if item_count < 10:  # Adjust the number as desired
#             root.geometry(f"400x{item_count * 80}")  # Adjust the height based on the item count

#         # Show/hide the scrollbar based on the content height
#         content_height = items_frame.winfo_height()
#         canvas_height = canvas.winfo_height()
#         if content_height <= canvas_height:
#             scrollbar.pack_forget()
#         else:
#             scrollbar.pack(side=RIGHT, fill=Y)

#     canvas.bind("<Configure>", on_configure)

#     def on_mousewheel(event):
#         canvas.yview_scroll(int(-1 * (event.delta / 120)), "units")

#     canvas.bind_all("<MouseWheel>", on_mousewheel)

#     submit_button = Button(root, text="Submit", command=submit, font=label_font)
#     submit_button.pack(side=BOTTOM, pady=20)

#     root.mainloop()

#     return lst

# def read_grocery_list(list_name="shoppinglist.txt"):
#     lst = []
#     with open(list_name) as file:
#         for line in file:
#             cur = line.split()
#             item_name = cur[0].lower()
#             amount = int(cur[1])
#             shelf_id = items_to_shelves[item_name]
#             if type(shelf_id) is not int:
#                 shelf_id = random.choice(shelf_id)
#             item = (item_name, amount, shelf_id) # item name, amount, shelf_id
#             lst.append(item)

#     return lst

# def enter_read_grocery_list(list_name):

#     def open_txt():
#         my_text.delete('1.0', END)
#         file = open(list_name, 'r')
#         text = file.read()

#         my_text.insert(END, text)
#         file.close()

#     def save_txt():
#         text_file = open(list_name, "w")
#         text_file.write(my_text.get(1.0, END).rstrip())
#         messagebox.showinfo("Info", "Grocery list was successfully saved!")
#         text_file.close()
        
#     def close():
#         root.quit()
    
#     root = Tk()
#     root.title("GroceryBee")
#     root.geometry("600x600")

#     my_text = scrolledtext.ScrolledText(root, width=40, height=10, font=("Arial Narrow", 14))
#     my_text.pack(pady=20)

#     open_button = Button(root, text="Open Grocery List", command=open_txt, font=("Arial Narrow", 14))
#     open_button.pack(pady=20)

#     save_button = Button(root, text="Save Grocery List", command=save_txt, font=("Arial Narrow", 14))
#     save_button.pack(pady=20)

#     Button(root, text="Exit", command=close, font=("Arial Narrow", 14)).pack(pady=20)

#     root.mainloop()

#     return read_grocery_list(list_name)
# print(enter_read_grocery_list("src/python/shoppinglist.txt"))
