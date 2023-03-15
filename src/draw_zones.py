import matplotlib.pyplot as plt 
import numpy as np 
from matplotlib.patches import Rectangle
import matplotlib.path as mplPath   
import math
from matplotlib.patches import Polygon

def rotate_vertices_90(vertices):
    # Define the transformation matrix
    matrix = np.array([[-1, 0], [0, 1]])
    
    # Apply the transformation to each vertex
    rotated_vertices = []
    for vertex in vertices:
        rotated_vertex = np.dot(matrix, vertex)
        rotated_vertices.append(rotated_vertex.tolist())
    print(rotated_vertices)
    return rotated_vertices
    
fig, ax = plt.subplots()   

vertices_arr_left = [[[0, 0], [1300, 0], [1300, 640], [1030, 1300], [-770, 600], [-280, -240], [0, 0]],
[[0, 0], [1300, 0], [1300, 640], [740, 2140], [-1100, 1000], [-280, -380], [0, 0]],
[[0, 0], [1300, 0], [1300, 640], [400, 2700], [-1500, 1300], [-280, -600], [0, 0]],
[[0, 0], [1300, 0], [1300, 640], [180, 3100], [-2100, 1500], [-280, -900], [0, 0]],
[[0, 0], [1300, 0], [1300, 640], [30, 3700], [-2700, 2000], [-280, -1100], [0, 0]]]
vertices_arr_right = [[[0, 0], [-1300, 0], [-1300, 640], [-1030, 1300], [770, 600], [280, -240], [0, 0]],
[[0, 0], [-1300, 0], [-1300, 640], [-740, 2140], [1100, 1000], [280, -380], [0, 0]],
[[0, 0], [-1300, 0], [-1300, 640], [-400, 2700], [1500, 1300], [280, -600], [0, 0]],
[[0, 0], [-1300, 0], [-1300, 640], [-180, 3100], [2100, 1500], [280, -900], [0, 0]],
[[0, 0], [-1300, 0], [-1300, 640], [-30, 3700], [2700, 2000], [280, -1100], [0, 0]]]

# for i in range(5):
#     rotate_vertices_90(vertices_arr_left[i])
vertices_arr_left_post = []
for i in range(5): 
    vertices = vertices_arr_left[i]
    for i in range(7):
        vertices[i][0] -= 350
    vertices_arr_left_post.append(vertices)
print('Real zones left: ',np.array(vertices_arr_left_post)/ 1000)
vertices_arr_right_post = []
for i in range(5): 
    vertices = vertices_arr_right[i]
    for i in range(7):
        vertices[i][0] += 350
    vertices_arr_right_post.append(vertices)
print('Real zones right: ',np.array(vertices_arr_right_post)/ 1000)

for i in range(5):
    
    vertices = vertices_arr_left_post[i]

    poly_path = mplPath.Path(np.array([vertices[0],
                                        vertices[1],
                                        vertices[2],
                                        vertices[3],
                                        vertices[4],
                                        vertices[5],
                                        vertices[0]]))
    point = (-1500, 500)
    x_coords, y_coords = zip(*vertices)
    
    ax.plot(x_coords, y_coords, ',',color=(1,0,0))
    
    ax.set_aspect('equal', adjustable='box')
    polygon = Polygon(vertices, closed=True)

    # Add the polygon to the plot
    ax.add_patch(polygon)

    # Set the fill color
    polygon.set_color((1, 0, 0,0.2))
    print(point, " is in polygon: ", poly_path.contains_point(point))

for i in range(5):
    
    vertices = vertices_arr_right_post[i]

    poly_path = mplPath.Path(np.array([vertices[0],
                                        vertices[1],
                                        vertices[2],
                                        vertices[3],
                                        vertices[4],
                                        vertices[5],
                                        vertices[0]]))
    point = (-1500, 500)
    x_coords, y_coords = zip(*vertices)
    
    ax.plot(x_coords, y_coords, ',',color=(0,0,1))
    ax.plot(point[0], point[1], '.',color=(0,0,0))
   
    
    ax.set_aspect('equal', adjustable='box')
    polygon = Polygon(vertices, closed=True)

    # Add the polygon to the plot
    ax.add_patch(polygon)

    # Set the fill color
    polygon.set_color((0, 0, 1,0.2))
    print(point, " is in polygon: ", poly_path.contains_point(point))
plt.xlabel('x [mm]')
plt.ylabel('y [mm]')
plt.title(label='Varnostne cone rahlo levo in rahlo desno')
plt.show()
    






    
