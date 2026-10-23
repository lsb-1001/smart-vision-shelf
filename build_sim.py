"""Generate a self-contained Gazebo Classic world, exact-layout map and registry."""
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

STATIONS = [('A',1,3.0,2.0,0.0), ('B',2,3.0,0.0,0.0),
            ('C',3,3.0,-2.0,0.0), ('DELIVERY',4,-3.0,0.0,math.pi)]


def box(name, pose, size, color, collision=True):
    geometry = f'<geometry><box><size>{size}</size></box></geometry>'
    visual = f'''<visual name="{name}"><pose>{pose}</pose>{geometry}
    <material><ambient>{color}</ambient><diffuse>{color}</diffuse></material></visual>'''
    if collision:
        visual += f'<collision name="{name}"><pose>{pose}</pose>{geometry}</collision>'
    return visual


def build(marker_grids, output):
    output = Path(output)
    output.mkdir(parents=True,exist_ok=True)
    elements = []
    for key, marker_id, x, y, yaw in STATIONS:
        visuals = [box('shelf','0.22 0 0.30 0 0 0','0.4 0.8 0.6','0.35 0.45 0.55 1'),
                   box('board','0 0 0.15 0 0 0','0.01 0.3 0.3','1 1 1 1')]
        cell = 0.24/6
        for row in range(6):
            for col in range(6):
                if marker_grids[marker_id][row][col] < 128:
                    cy,cz = (2.5-col)*cell, 0.15+(2.5-row)*cell
                    visuals.append(box(f'cell_{row}_{col}',f'-0.006 {cy} {cz} 0 0 0',
                                       f'0.002 {cell} {cell}','0 0 0 1',False))
        elements.append(f'<model name="station_{key}"><static>true</static>'
                        f'<pose>{x} {y} 0 0 0 {yaw}</pose><link name="body">'
                        +''.join(visuals)+'</link></model>')
    walls = [('north','0 4 0.5 0 0 0','8.1 0.1 1'),
             ('south','0 -4 0.5 0 0 0','8.1 0.1 1'),
             ('east','4 0 0.5 0 0 0','0.1 8.1 1'),
             ('west','-4 0 0.5 0 0 0','0.1 8.1 1')]
    for name,pose,size in walls:
        elements.append(f'<model name="wall_{name}"><static>true</static><link name="wall">'
                        +box('wall',pose,size,'0.65 0.65 0.65 1')+'</link></model>')
    world = '''<sdf version="1.6"><world name="shelf_demo">
    <include><uri>model://sun</uri></include>
    <include><uri>model://ground_plane</uri></include>
    <physics type="ode"><max_step_size>0.001</max_step_size><real_time_update_rate>1000</real_time_update_rate></physics>
    <scene><ambient>0.6 0.6 0.6 1</ambient><shadows>false</shadows></scene>
    <gui><camera name="overview"><pose>0 -7 9 0 0.85 1.57</pose></camera></gui>
    '''+''.join(elements)+'</world></sdf>'
    ET.fromstring(world)
    (output/'shelf_demo.world').write_text(world,encoding='utf-8')
    # Map is generated from the same stationary collision boxes, not a SLAM measurement.
    resolution, width, height, origin = 0.05, 168, 168, -4.2
    pixels = bytearray()
    def occupied(x,y):
        if abs(x) >= 3.925 or abs(y) >= 3.925:
            return True
        for _,_,sx,sy,yaw in STATIONS:
            dx,dy = x-sx,y-sy
            local_x = math.cos(yaw)*dx+math.sin(yaw)*dy
            local_y = -math.sin(yaw)*dx+math.cos(yaw)*dy
            if 0.0 <= local_x <= 0.45 and abs(local_y) <= 0.425:
                return True
            if abs(local_x) <= 0.03 and abs(local_y) <= 0.175:
                return True
        return False
    for image_y in range(height):
        y = origin+(height-image_y-0.5)*resolution
        for image_x in range(width):
            x = origin+(image_x+0.5)*resolution
            pixels.append(0 if occupied(x,y) else 254)
    (output/'shelf_demo.pgm').write_bytes(f'P5\n{width} {height}\n255\n'.encode()+pixels)
    (output/'shelf_demo.yaml').write_text(
        'image: shelf_demo.pgm\nmode: trinary\nresolution: 0.05\n'
        'origin: [-4.2, -4.2, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n',
        encoding='utf-8')
    registry = json.loads(Path(__file__).with_name('locations.json').read_text(encoding='utf-8'))
    registry.update(map_id='gazebo_shelf_demo_v1',simulation_only=True)
    for entry in registry['locations']:
        _,_,x,y,yaw = next(s for s in STATIONS if s[0] == entry['key'])
        entry['approach_pose'] = dict(frame_id='map',x=round(x-0.8*math.cos(yaw),4),
                                    y=round(y-0.8*math.sin(yaw),4),yaw=yaw)
    (output/'locations.sim.json').write_text(json.dumps(registry,indent=2),encoding='utf-8')
    (output/'BUILD_INFO.json').write_text(json.dumps(dict(
        simulation_only=True,marker_black_side_m=0.24,map_source='generated_from_world_geometry',
        robot_spawn=dict(x=2.2,y=0.0,yaw=0.0),
        note='Not physical-map coordinates; marker dictionary still provisional'),indent=2),encoding='utf-8')
    return output


def main():
    import cv2
    from shelf_view import dictionary, marker_image
    registry = json.loads(Path(__file__).with_name('locations.json').read_text(encoding='utf-8'))
    book = dictionary(cv2,registry['dictionary'])
    grids = {}
    for _,marker_id,*_ in STATIONS:
        marker = marker_image(cv2,book,marker_id,600)
        grids[marker_id] = [[int(marker[r*100+50,c*100+50]) for c in range(6)] for r in range(6)]
    output = build(grids,Path(__file__).with_name('sim'))
    print('Built simulation assets:',output)
    print('Spawn: Shelf B approach (2.2, 0), facing ID 2. No robot motion yet.')


if __name__ == '__main__':
    main()
