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

# vertices_arr_left = [[[0,0],[800,0],[800,600],[-800,600],[-800,0],[0,0]],
# [[0, 0], [1300, 0], [1300, 640], [1030, 1300], [-770, 600], [-280, -240], [0, 0]],
# [[0, 0], [1300, 0], [1300, 640], [740, 2140], [-1100, 1000], [-280, -380], [0, 0]],
# [[0, 0], [1300, 0], [1300, 640], [400, 2700], [-1500, 1300], [-280, -600], [0, 0]],
# [[0, 0], [1300, 0], [1300, 640], [180, 3100], [-2100, 1500], [-280, -900], [0, 0]],
# [[0, 0], [1300, 0], [1300, 640], [30, 3700], [-2700, 2000], [-280, -1100], [0, 0]]]
# vertices_arr_right = [[[0,0],[800,0],[800,600],[-800,600],[-800,0],[0,0]],
# [[0, 0], [-1300, 0], [-1300, 640], [-1030, 1300], [770, 600], [280, -240], [0, 0]],
# [[0, 0], [-1300, 0], [-1300, 640], [-740, 2140], [1100, 1000], [280, -380], [0, 0]],
# [[0, 0], [-1300, 0], [-1300, 640], [-400, 2700], [1500, 1300], [280, -600], [0, 0]],
# [[0, 0], [-1300, 0], [-1300, 640], [-180, 3100], [2100, 1500], [280, -900], [0, 0]],
# [[0, 0], [-1300, 0], [-1300, 640], [-30, 3700], [2700, 2000], [280, -1100], [0, 0]]]
# vertices_arr_forward = [[[0,0],[800,0],[800,600],[-800,600],[-800,0],[0,0]],
# [[0,0],[850,0],[850,1000],[-850,1000],[-850,0],[0,0]],
# [[0,0],[1000,0],[1000,1700],[-1000,1700],[-1000,0],[0,0]],
# [[0,0],[1100,0],[1100,2500],[-1100,2500],[-1100,0],[0,0]],
# [[0,0],[1200,0],[1200,3200],[-1200,3200],[-1200,0],[0,0]],
# [[0,0],[1250,0],[1250,3800],[-1250,3800],[-1300,0],[0,0]]]

# # for i in range(5):
# #     rotate_vertices_90(vertices_arr_left[i])
# vertices_arr_left_post = [vertices_arr_left[0]]
# for i in range(1,len(vertices_arr_left)): 
#     vertices = vertices_arr_left[i]
#     for i in range(len(vertices)):
#         vertices[i][0] -= 350
#     vertices_arr_left_post.append(vertices)

# lst = [[[x/1000, y/1000] for x, y in inner_lst] for inner_lst in vertices_arr_left_post]
# print(lst)

# vertices_arr_right_post = [vertices_arr_right[0]]
# for i in range(1,len(vertices_arr_left)): 
#     vertices = vertices_arr_right[i]
#     for i in range(len(vertices)):
#         vertices[i][0] += 350
#     vertices_arr_right_post.append(vertices)

# lst = [[[x/1000, y/1000] for x, y in inner_lst] for inner_lst in vertices_arr_right_post]
# print(lst)

# lst = [[[x/1000, y/1000] for x, y in inner_lst] for inner_lst in vertices_arr_forward]
# print(lst)

vertices_arr_left_post = [[[0.0, 0.0], [0.8, 0.0], [0.8, 0.6], [-0.8, 0.6], [-0.8, 0.0], [0.0, 0.0]], [[-0.35, 0.0], [0.95, 0.0], [0.95, 0.64], [0.68, 1.3], [-1.12, 0.6], [-0.63, -0.24], [-0.35, 0.0]], [[-0.35, 0.0], [0.95, 0.0], [0.95, 0.64], [0.39, 2.14], [-1.45, 1.0], [-0.63, -0.38], [-0.35, 0.0]], [[-0.35, 0.0], [0.95, 0.0], [0.95, 0.64], [0.05, 2.7], [-1.85, 1.3], [-0.63, -0.6], [-0.35, 0.0]], [[-0.35, 0.0], [0.95, 0.0], [0.95, 0.64], [-0.17, 3.1], [-2.45, 1.5], [-0.63, -0.9], [-0.35, 0.0]], [[-0.35, 0.0], [0.95, 0.0], [0.95, 0.64], [-0.32, 3.7], [-3.05, 2.0], [-0.63, -1.1], [-0.35, 0.0]]]
vertices_arr_right_post = [[[0.0, 0.0], [0.8, 0.0], [0.8, 0.6], [-0.8, 0.6], [-0.8, 0.0], [0.0, 0.0]], [[0.35, 0.0], [-0.95, 0.0], [-0.95, 0.64], [-0.68, 1.3], [1.12, 0.6], [0.63, -0.24], [0.35, 0.0]], [[0.35, 0.0], [-0.95, 0.0], [-0.95, 0.64], [-0.39, 2.14], [1.45, 1.0], [0.63, -0.38], [0.35, 0.0]], [[0.35, 0.0], [-0.95, 0.0], [-0.95, 0.64], [-0.05, 2.7], [1.85, 1.3], [0.63, -0.6], [0.35, 0.0]], [[0.35, 0.0], [-0.95, 0.0], [-0.95, 0.64], [0.17, 3.1], [2.45, 1.5], [0.63, -0.9], [0.35, 0.0]], [[0.35, 0.0], [-0.95, 0.0], [-0.95, 0.64], [0.32, 3.7], [3.05, 2.0], [0.63, -1.1], [0.35, 0.0]]]
vertices_arr_forward = [[[0.0, 0.0], [0.8, 0.0], [0.8, 0.6], [-0.8, 0.6], [-0.8, 0.0], [0.0, 0.0]], [[0.0, 0.0], [0.85, 0.0], [0.85, 1.0], [-0.85, 1.0], [-0.85, 0.0], [0.0, 0.0]], [[0.0, 0.0], [1.0, 0.0], [1.0, 1.7], [-1.0, 1.7], [-1.0, 0.0], [0.0, 0.0]], [[0.0, 0.0], [1.1, 0.0], [1.1, 2.5], [-1.1, 2.5], [-1.1, 0.0], [0.0, 0.0]], [[0.0, 0.0], [1.2, 0.0], [1.2, 3.2], [-1.2, 3.2], [-1.2, 0.0], [0.0, 0.0]], [[0.0, 0.0], [1.25, 0.0], [1.25, 3.8], [-1.25, 3.8], [-1.3, 0.0], [0.0, 0.0]]]

global point
point = (-1.500, 0.500)
for i in range(6):
    vertices = vertices_arr_left_post[i]

    poly_path = mplPath.Path(np.array([vertices[0],
                                        vertices[1],
                                        vertices[2],
                                        vertices[3],
                                        vertices[4],
                                        vertices[5],
                                        vertices[0]]))

    x_coords, y_coords = zip(*vertices)
    
    ax.plot(x_coords, y_coords, ',',color=(1,0,0))
    
    ax.set_aspect('equal', adjustable='box')
    polygon = Polygon(vertices, closed=True)

    # Add the polygon to the plot
    ax.add_patch(polygon)

    # Set the fill color
    polygon.set_color((1, 0, 0,0.2))
    print(point, " is in polygon: ", poly_path.contains_point(point))

for i in range(6):

    vertices = vertices_arr_right_post[i]

    poly_path = mplPath.Path(np.array([vertices[0],
                                        vertices[1],
                                        vertices[2],
                                        vertices[3],
                                        vertices[4],
                                        vertices[5],
                                        vertices[0]]))
    x_coords, y_coords = zip(*vertices)
    
    ax.plot(x_coords, y_coords, ',',color=(0,0,1))
   
    ax.set_aspect('equal', adjustable='box')
    polygon = Polygon(vertices, closed=True)

    # Add the polygon to the plot
    ax.add_patch(polygon)

    # Set the fill color
    polygon.set_color((0, 0, 1,0.2))
    print(point, " is in polygon: ", poly_path.contains_point(point))

for i in range(6):

    vertices = vertices_arr_forward[i]

    poly_path = mplPath.Path(np.array([vertices[0],
                                        vertices[1],
                                        vertices[2],
                                        vertices[3],
                                        vertices[4],
                                        vertices[5],
                                        vertices[0]]))
    
    x_coords, y_coords = zip(*vertices)
    
    ax.plot(x_coords, y_coords, ',',color=(0,1,0))
    ax.plot(point[0], point[1], '.',color=(0,0,0))
   
    
    ax.set_aspect('equal', adjustable='box')
    polygon = Polygon(vertices, closed=True)

    # Add the polygon to the plot
    ax.add_patch(polygon)

    # Set the fill color
    polygon.set_color((0, 1, 0,0.2))
    print(point, " is in polygon: ", poly_path.contains_point(point))


plt.xlabel('x [mm]')
plt.ylabel('y [mm]')
plt.title(label='Varnostne cone rahlo levo in rahlo desno')
plt.show()
    






    
