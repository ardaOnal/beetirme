import random
# shelf index:  1  item:  <RigidBody_[float] name='base_link_cracker' index=69 model_instance=51>
# shelf index:  2  item:  <RigidBody_[float] name='base_link_sugar' index=72 model_instance=54>
# shelf index:  3  item:  <RigidBody_[float] name='base_link_soup' index=75 model_instance=57>
# shelf index:  4  item:  <RigidBody_[float] name='base_link_mustard' index=78 model_instance=60>
# shelf index:  5  item:  <RigidBody_[float] name='base_link_gelatin' index=81 model_instance=63>
# shelf index:  6  item:  <RigidBody_[float] name='base_link_meat' index=84 model_instance=66>
# shelf index:  7  item:  <RigidBody_[float] name='base_link_cracker' index=87 model_instance=69>
# shelf index:  8  item:  <RigidBody_[float] name='base_link_sugar' index=90 model_instance=72>
# shelf index:  9  item:  <RigidBody_[float] name='base_link_soup' index=93 model_instance=75>

items_to_shelves = {"cracker": (1, 7), "sugar": (2, 8), "soup": (3, 9), "mustard": (4), "jello": (5), "meat": (6)}

def read_grocery_list(list_name):
    lst = []
    with open(list_name) as file:
        for line in file:
            cur = line.split()
            item_name = cur[0].lower()
            amount = cur[1]
            shelf_id = items_to_shelves[item_name]
            if type(shelf_id) is not int:
                shelf_id = random.choice(shelf_id)
            item = (item_name, amount, shelf_id) # item name, amount, shelf_id
            lst.append(item)

    return lst

print(read_grocery_list("src/python/shoppinglist.txt"))
