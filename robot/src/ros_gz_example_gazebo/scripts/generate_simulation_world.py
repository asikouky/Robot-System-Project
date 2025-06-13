import argparse
import random
import math
import xml.etree.ElementTree as ET

def distance_between_segments(seg1_start, seg1_end, seg2_start, seg2_end):    

    def point_to_segment_distance(p, s, e):
        # Vector from s to e
        segment = (e[0] - s[0], e[1] - s[1])
        # Vector from s to p
        point = (p[0] - s[0], p[1] - s[1])
        
        # Calculate projection of point onto segment
        segment_length_sq = segment[0]**2 + segment[1]**2
        
        # Handle case where segment is a point
        if segment_length_sq == 0:
            return math.sqrt((p[0] - s[0])**2 + (p[1] - s[1])**2)
        
        # Calculate projection ratio
        t = max(0, min(1, (point[0]*segment[0] + point[1]*segment[1]) / segment_length_sq))
        
        # Calculate closest point on segment
        projection = (s[0] + t * segment[0], s[1] + t * segment[1])
        
        # Return distance from p to projection
        return math.sqrt((p[0] - projection[0])**2 + (p[1] - projection[1])**2)
    
    d1 = point_to_segment_distance(seg1_start, seg2_start, seg2_end)
    d2 = point_to_segment_distance(seg1_end, seg2_start, seg2_end)
    d3 = point_to_segment_distance(seg2_start, seg1_start, seg1_end)
    d4 = point_to_segment_distance(seg2_end, seg1_start, seg1_end)
    
    return min(d1, d2, d3, d4)

def add_random_walls(world, max_attempts=100):
    min_x = -6
    max_x = 6
    z = 0.5
    min_y = -6
    max_y = 6
    wall_length = 5
    wall_height = 1
    wall_width = 0.2
    
    min_separation = 1.5
    
    placed_walls = [] # (x, y, yaw, corners)
    
    def calculate_wall_corners(x, y, yaw):
        half_length = wall_length / 2
        half_width = wall_width / 2
        
        corners_rel = [
            (-half_length, -half_width),
            (half_length, -half_width),
            (half_length, half_width),
            (-half_length, half_width)
        ]
        
        corners = []
        for cx, cy in corners_rel:
            rx = cx * math.cos(yaw) - cy * math.sin(yaw)
            ry = cx * math.sin(yaw) + cy * math.cos(yaw)
            corners.append((x + rx, y + ry))
        
        return corners
    
    def walls_too_close(new_wall_x, new_wall_y, new_wall_yaw, existing_walls):
        new_corners = calculate_wall_corners(new_wall_x, new_wall_y, new_wall_yaw)
        
        new_segments = [(new_corners[i], new_corners[(i+1)%4]) for i in range(4)]
        
        for wall_x, wall_y, wall_yaw, corners in existing_walls:
            wall_segments = [(corners[i], corners[(i+1)%4]) for i in range(4)]
            
            for new_seg in new_segments:
                for wall_seg in wall_segments:
                    dist = distance_between_segments(new_seg[0], new_seg[1], wall_seg[0], wall_seg[1])
                    if dist < min_separation:
                        return True
        
        return False
    
    for i in range(3):
        wall_placed = False
        attempts = 0
        
        while not wall_placed and attempts < max_attempts:
            wall_x = random.uniform(min_x + wall_length/2, max_x - wall_length/2)
            wall_y = random.uniform(min_y + wall_length/2, max_y - wall_length/2)
            
            yaw = random.uniform(0, 2 * math.pi)
            
            corners = calculate_wall_corners(wall_x, wall_y, yaw)
            
            # Check if this wall is too close to any existing walls
            if not walls_too_close(wall_x, wall_y, yaw, placed_walls):

                placed_walls.append((wall_x, wall_y, yaw, corners))
                wall_placed = True
                
                wall = ET.SubElement(world, 'model', name=f'wall_{i}')
                ET.SubElement(wall, 'static').text = 'true'
                ET.SubElement(wall, 'pose').text = f'{wall_x} {wall_y} {z} 0 0 {yaw}'
                
                link = ET.SubElement(wall, 'link', name='link')
                collision = ET.SubElement(link, 'collision', name='collision')
                geometry = ET.SubElement(collision, 'geometry')
                box = ET.SubElement(geometry, 'box')
                ET.SubElement(box, 'size').text = f'{wall_length} {wall_width} {wall_height}'
                
                visual = ET.SubElement(link, 'visual', name='visual')
                geometry = ET.SubElement(visual, 'geometry')
                box = ET.SubElement(geometry, 'box')
                ET.SubElement(box, 'size').text = f'{wall_length} {wall_width} {wall_height}'
                
                material = ET.SubElement(visual, 'material')
                ET.SubElement(material, 'ambient').text = '0.8 0.8 0.8 1'
                ET.SubElement(material, 'diffuse').text = '0.8 0.8 0.8 1'
                ET.SubElement(material, 'specular').text = '0.8 0.8 0.8 1'
                
                print(f"Wall {i} placed at ({wall_x:.2f}, {wall_y:.2f}) with rotation {yaw:.2f} radians")
            
            attempts += 1
        
        if not wall_placed:
            print(f"Warning: Could not place wall {i} after {max_attempts} attempts")

def main():
    print("Generating simulation world with non-overlapping walls")
    
    parser = argparse.ArgumentParser(description='Generate a Gazebo simulation world file')
    parser.add_argument('--sdf-source', type=str, required=True, help='Path to the input SDF file')
    parser.add_argument('--sdf-dest', type=str, required=True, help='Path to the output SDF file')
    args = parser.parse_args()
    
    tree = ET.parse(args.sdf_source)
    root = tree.getroot()
    world = root.find('world')
    
    add_random_walls(world)
    
    tree.write(args.sdf_dest, encoding='utf-8', xml_declaration=True)
    print(f"World file saved to {args.sdf_dest}")

if __name__ == "__main__":
    main()