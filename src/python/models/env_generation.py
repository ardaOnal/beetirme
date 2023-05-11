def grid(row_count=5, row_start_point=-6.6, row_increment=3.3, shelf_row_count=5, shelf_start_point=-2.2, shelf_increment=1.1):
    f = open("src/python/models/shelves.dmd.yaml", "w")

    row_no = 1
    shelf_no = 1
    model_directives = "directives:"

    cur_row = row_start_point

    while row_no <= row_count:
        cur_shelves = 1
        cur_point = shelf_start_point
        while cur_shelves <= shelf_row_count:
            name = "shelves" + str(shelf_no)
            origin = name + "_origin"
            
            model_directives += f"""
- add_frame:
    name: {origin}
    X_PF:
        base_frame: world
        translation: [{cur_point}, {cur_row}, 0.6085]
        rotation: !Rpy {{ deg: [0, 0, 90]}}

- add_model:
    name: {name}
    file: package://grocery/shelves.sdf

- add_weld:
    parent: {origin}
    child: {name}::shelves_body

- add_frame:
    name: camera0_{shelf_no}_origin
    X_PF:
        base_frame: {origin}
        rotation: !Rpy {{ deg: [-120,0,240]}}
        translation: [-0.35,.25,0.05]
- add_model:
    name: camera0_{shelf_no}
    file: package://grocery/camera_box.sdf
- add_weld:
    parent: camera0_{shelf_no}_origin
    child: camera0_{shelf_no}::base

- add_frame:
    name: camera1_{shelf_no}_origin
    X_PF:
        base_frame: {origin}
        rotation: !Rpy {{ deg: [-120, 0, -55]}}
        translation: [-.3, -.33, 0.05]
- add_model:
    name: camera1_{shelf_no}
    file: package://grocery/camera_box.sdf
- add_weld:
    parent: camera1_{shelf_no}_origin
    child: camera1_{shelf_no}::base
"""
            cur_point += shelf_increment
            shelf_no += 1
            cur_shelves += 1

        cur_row += row_increment
        row_no += 1

    f.write(model_directives)
    f.close()




def maze(start=3, side=3, increment=-1):
    f = open("src/python/models/shelves.dmd.yaml", "w")
    model_directives = "directives:"

    neg_xcoordinate = -start
    cur_side = 1
    cur_coordinate = start
    shelf_no = 1
    while cur_side <= side:
        name = "shelves" + str(shelf_no)
        origin = name + "_origin"
        model_directives += f"""
- add_frame:
    name: {origin}
    X_PF:
        base_frame: world
        translation: [{start}, {cur_coordinate}, 0.6085]
        rotation: !Rpy {{ deg: [0, 0, 0]}}

- add_model:
    name: {name}
    file: package://grocery/shelves.sdf

- add_weld:
    parent: {origin}
    child: {name}::shelves_body
"""
        shelf_no += 1
        name = "shelves" + str(shelf_no)
        origin = name + "_origin"
        model_directives += f"""
- add_frame:
    name: {origin}
    X_PF:
        base_frame: world
        translation: [{neg_xcoordinate}, {cur_coordinate}, 0.6085]
        rotation: !Rpy {{ deg: [0, 0, 0]}}

- add_model:
    name: {name}
    file: package://grocery/shelves.sdf

- add_weld:
    parent: {origin}
    child: {name}::shelves_body
"""
        cur_coordinate += increment
        shelf_no += 1
        cur_side += 1


    cur_side = 1
    cur_x = start
    while cur_side <= side:
        name = "shelves" + str(shelf_no)
        origin = name + "_origin"
        model_directives += f"""
- add_frame:
    name: {origin}
    X_PF:
        base_frame: world
        translation: [{cur_x}, {cur_coordinate}, 0.6085]
        rotation: !Rpy {{ deg: [0, 0, 90]}}

- add_model:
    name: {name}
    file: package://grocery/shelves.sdf

- add_weld:
    parent: {origin}
    child: {name}::shelves_body
"""
        cur_side += 1
        cur_x += increment
        shelf_no +=1


    f.write(model_directives)
    f.close()

grid()
