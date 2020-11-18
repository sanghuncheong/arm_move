import numpy as np
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d

# Create data
def make_box_data_old(x_min, x_max, x_num, y_min, y_max, y_num, z_min, z_max, z_num, z_rotation):
    x_d = np.linspace(x_min,x_max,x_num)
    y_d = np.linspace(y_min,y_max,y_num)
    z_d = np.linspace(z_min,z_max,z_num)

    x_data=[]
    y_data=[]
    z_data=[]

    x_center = (x_min+x_max)/2
    y_center = (y_min+y_max)/2
    for z_cont in range(z_num):
        for x_cont in range(x_num):
            for y_cont in range(y_num):
                x_data.append(x_center + math.cos(z_rotation)*(x_d[x_cont]-x_center) - math.sin(z_rotation)*(y_d[y_cont]-y_center))
                y_data.append(y_center + math.sin(z_rotation)*(x_d[x_cont]-x_center) + math.cos(z_rotation)*(y_d[y_cont]-y_center))
                z_data.append(z_d[z_cont])

    data_set = [x_data,y_data,z_data]
    return data_set

def make_box_data(x_center, y_center,z_center, width, depth, height, z_rotation , density=1):

    x_min = x_center - width / 2
    x_max = x_center + width / 2
    y_min = y_center - depth / 2
    y_max = y_center + depth / 2
    z_min = z_center - height / 2
    z_max = z_center + height / 2

    x_num = int((x_max-x_min)*100*density)+1
    y_num = int((y_max-y_min)*100*density)+1
    z_num = int((z_max-z_min)*100*density)+1

    print x_num,y_num,z_num

    x_d = np.linspace(x_min,x_max,x_num)
    y_d = np.linspace(y_min,y_max,y_num)
    z_d = np.linspace(z_min,z_max,z_num)

    x_data=[]
    y_data=[]
    z_data=[]

    x_center = (x_min+x_max)/2
    y_center = (y_min+y_max)/2

    for z_cont in range(z_num):
        for x_cont in range(x_num):
            for y_cont in range(y_num):
                x_data.append(x_center + math.cos(z_rotation)*(x_d[x_cont]-x_center) - math.sin(z_rotation)*(y_d[y_cont]-y_center))
                y_data.append(y_center + math.sin(z_rotation)*(x_d[x_cont]-x_center) + math.cos(z_rotation)*(y_d[y_cont]-y_center))
                z_data.append(z_d[z_cont])

    data_set = [x_data,y_data,z_data]
    return data_set
def make_cylinder(x_center,y_center,z_center,radius,height,density):
    theta = np.linspace(0,2*3.14,density)
    c_theta = len(theta)
    z_d = np.linspace(0, height, density)

    x_data = []
    y_data = []
    z_data = []

    for z_cont in range(density):
      for i in range(c_theta):
          x_data.append(x_center + radius*math.cos(theta[i]))
          y_data.append(y_center + radius*math.sin(theta[i]))
          z_data.append(z_center + z_d[z_cont])

    data_set = [x_data, y_data, z_data]
    return data_set

def draw_object(data,color,size,alpha):
    global fig,ax
    x, y, z = data
    ax.scatter(x,y,z, alpha = alpha, c = color, edgecolors='none', s = size)

global fig,ax

table_z_rotation = 0

#workspace_data      = make_box_data1(0, 3, 10, 0, 3, 0.10, 0, 3, 0.10, 0)
#shelf_floor_1_data  = make_box_data1(0, 0.3, 10, 0, 0.80, 0.10, 0.80 + 0.30 * 0, 0.81 + 0.3 * 0, 0.02, 0)
#shelf_floor_2_data  = make_box_data1(0, 0.3, 10, 0, 0.80, 0.10, 0.80 + 0.30 * 1, 0.81 + 0.3 * 1, 0.02, 0)
#shelf_floor_3_data  = make_box_data1(0, 0.3, 10, 0, 0.80, 0.10, 0.80 + 0.30 * 2, 0.81 + 0.3 * 2, 0.02, 0)
#table_data          = make_box_data1(0.3, 0.3 + 1.4, 0.2, 0, 0.8, 0.1, 0.8, 0.81, 0.02, z_rotation=table_z_rotation)    #rotation = radian!
workspace_data      = make_box_data(x_center=1.50,y_center=1.5 ,z_center=1.5    ,width=3.0 ,depth=3.0,height=3   ,z_rotation=0,     density=0.03 )
shelf_floor_1_data  = make_box_data(x_center=0.15,y_center=0.4 ,z_center=0.8+0.0,width=0.8 ,depth=0.3,height=0.10,z_rotation=3.14/2,density=0.4  )
shelf_floor_2_data  = make_box_data(x_center=0.15,y_center=0.4 ,z_center=0.8+0.5,width=0.8 ,depth=0.3,height=0.10,z_rotation=3.14/2,density=0.4  )
shelf_floor_3_data  = make_box_data(x_center=0.15,y_center=0.4 ,z_center=0.8+1.0,width=0.8 ,depth=0.3,height=0.10,z_rotation=3.14/2,density=0.4  )
table_data          = make_box_data(x_center=1.00,y_center=0.40,z_center=0.80   ,width=1.4 ,depth=0.8,height=0.10,z_rotation=0,     density=0.4  )
book_data           = make_box_data(x_center=0.70,y_center=0.40,z_center=0.90   ,width=0.40,depth=0.2,height=0.05,z_rotation=3.14/4,density=0.7  )
cup_1_data          = make_cylinder(x_center=1.50, y_center=0.20, z_center=0.8+0.05+0.075,      radius=0.05, height=0.15, density=10)
cup_2_data          = make_cylinder(x_center=0.15, y_center=0.40, z_center=0.80+0.5+0.05+0.075, radius=0.05, height=0.15, density=10)

# Create plot
fig = plt.figure()
ax = fig.add_subplot(1, 1, 1, axisbg="1.0")
ax = fig.gca(projection='3d')

draw_object(workspace_data,     'gray',         size=20,alpha=0.2)
draw_object(shelf_floor_1_data, 'darkgoldenrod',size=30,alpha=0.3)
draw_object(shelf_floor_2_data, 'darkgoldenrod',size=30,alpha=0.3)
draw_object(shelf_floor_3_data, 'darkgoldenrod',size=30,alpha=0.3)
draw_object(table_data,         'olive',        size=30,alpha=0.3)
draw_object(book_data,          'indigo',       size=50,alpha=0.8)
draw_object(cup_1_data,           'blue',       size=50,alpha=0.8)
draw_object(cup_2_data,           'blue',       size=50,alpha=0.8)


#print book_data
#x, y, z = data
#ax.scatter(x, y, z, alpha=0.8, c=colors, edgecolors='none', s=100, label=groups)


#for data, color, group in zip(data, colors, groups):
#    x, y, z = data
#    ax.scatter(x, y, z, alpha=0.8, c=color, edgecolors='none', s=30, label=group)


#print x
#print y
#print z
plt.title('Matplot 3d scatter plot')
plt.legend(loc=2)
plt.show()
