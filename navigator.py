import vtk
import time as t
import numpy as np

from vtkmodules.vtkFiltersSources import vtkPlaneSource
from vtkmodules.vtkFiltersCore import vtkFeatureEdges
from vtkmodules.vtkFiltersSources import vtkRegularPolygonSource
from vtkmodules.vtkRenderingCore import (
    vtkActor,
    vtkPolyDataMapper,
)
from vtkmodules.vtkCommonColor import vtkNamedColors

move_step = 10
speed = 640
e_sftlck_state = False
scale = 0.75
override_pos_limits_state = 0
user_axis_control = True
i = 0
c = 100
cnc_mode_state = False
show_volume_bounds = True
cmm_position = [] 

global_volumetric_limits = [230,      # -X
                            775,      # +X
                            170,    # -Y
                            1200,     # +Y
                            600,      # -Z
                            1130]     # +Z

local_volumetric_limits = [0        ,     # -X
                           755      ,     # +X
                           -1005  ,     # -Y
                           5        ,     # +Y
                           -540        ,     # -Z
                           5]             # +Z

actor1_position = [0,0,0]                               ## desempeno

actor4_position = [315,                                 ## cabeçote
                   1039.5,
                   1080]                                #### POSIÇÃO INICIAL EM Z

actor3_position = [actor4_position[0]   -   310,              ## ponte
                   actor4_position[1]   +   60,                           #### POSIÇÃO INICIAL EM Y
                   actor4_position[2]   -   480]

actor2_position = [actor4_position[0]   -   150,            ## capa do z    #### POSIÇÃO INICIAL EM X
                   actor4_position[1]   +   360,
                   actor4_position[2]   +   180]

actor5_position = [1000,                                ## esfera
                   1000, 
                   actor3_position[2]   +   210]

local_safe_limits = [local_volumetric_limits[0]     +   5      ,   ## deslocamento automático após e_sftlck_state False em check_local_volumetric_limits
                     local_volumetric_limits[1]     -   5      ,
                     local_volumetric_limits[2]     +   5      ,
                     local_volumetric_limits[3]     -   5      ,
                     local_volumetric_limits[4]     +   5      ,
                     local_volumetric_limits[5]     -   5      
                     ]
                
local_origin = np.array([global_volumetric_limits[0]    -3      *     move_step, 
                         global_volumetric_limits[3]    -3      *     move_step, 
                         global_volumetric_limits[5]    -3      *     move_step])

local_axes = np.identity(3)

def get_global_current_position(actor4_position):
    return actor4_position

def get_local_current_position(local_axes,local_origin,actor):            ## função para calcular posição atual em relação ao sistema de coordenadas da máquina
    global_current_position = get_global_current_position(actor)
    return np.linalg.inv(local_axes) @ (global_current_position - local_origin)             ## trata-se da inversa das coordenadas globais da origem do sistema secundário multiplicado
                                                                                            ## pela distância (por eixo) entre as coordenadas globais locais

def user_axis_control_check():
    return user_axis_control

def lock_notice():                                                      ## pública para ser acessada a qualquer momento
    print('LOCKING\nLOCAL POSITION: ',get_local_current_position(local_axes,local_origin,actor4_position))
    global user_axis_control
    user_axis_control = False

def command_restore():
    global cnc_mode_state
    if cnc_mode_state == True:
       cnc_mode_state = False 
    else:
        cnc_mode_state = True
    print('>>> COMMANDS', cnc_mode_state)

def cnc_mode_switch():
    global cnc_mode_state
    if cnc_mode_state == True:
       cnc_mode_state = False 
    else:
        cnc_mode_state = True
    print('CNC MODE', cnc_mode_state)
    return cnc_mode_state

def check_cnc_position_list(cmm_position,type):
    cmm_position_list_length = len(cmm_position)
    
    if cmm_position_list_length < 2 and type == 1:                  #TYPE 1: LINE
        return 0
    elif cmm_position_list_length < 3 and type == 2:                #TYPE 2: PLANE
        return 0
    
    else:
        return cmm_position

def e_sftlck_state_unlock(actor4_position):
    global e_sftlck_state, user_axis_control
    entrada = input('TYPE "Y" TO UNLOCK: ')

    if entrada.strip().upper() == "Y":
        e_sftlck_state = False
        user_axis_control = True
        entrada = ''
        print('>>> UNLOCKED')
    else:
        entrada = ''
        print('>>> UNLOCKING CANCELED\n>>> USER AXIS CONTROL False')

def e_sftlck_state_lock(actor4_position):
    lock_notice()    
    print('LOCAL VOLUMETRIC LIMITS ',local_volumetric_limits)
    global e_sftlck_state
    e_sftlck_state = True
    print('>>> LOCKED')
    e_sftlck_state_unlock(actor4_position)

def check_global_volumetric_limits(actor4_position):
    if actor4_position[0] < global_volumetric_limits[0]:                                      ## verifica limite -X
        print('>>> LIMIT -X')
        e_sftlck_state_lock(actor4_position)
        return 1
    elif actor4_position[0] > global_volumetric_limits[1]:                                    ## verifica limite +X
        print('>>> LIMIT +X')
        e_sftlck_state_lock(actor4_position)
        return 1
    elif actor4_position[1] < global_volumetric_limits[2]:                                    ## verifica limite -Y
        print('>>> LIMIT -Y')
        e_sftlck_state_lock(actor4_position)
        return 1
    elif actor4_position[1] > global_volumetric_limits[3]:                                    ## verifica limite +Y
        print('>>> LIMIT +Y')
        e_sftlck_state_lock(actor4_position)
        return 1
    elif actor4_position[2] < global_volumetric_limits[4]:                                    ## verifica limite -Z
        print('>>> LIMIT -Z')
        e_sftlck_state_lock(actor4_position)
        return 1
    elif actor4_position[2] > global_volumetric_limits[5]:                                    ## verifica limite +Z
        print('>>> LIMIT +Z')
        e_sftlck_state_lock(actor4_position)
        return 1

def check_local_volumetric_limits(local_axes,local_origin):
    if get_local_current_position(local_axes,local_origin,actor4_position)[0] < local_volumetric_limits[0]:                                      ## verifica limite local -X
        print('>>> LOCAL LIMIT -X')
        cnc_mode_switch()
        return 1
    elif get_local_current_position(local_axes,local_origin,actor4_position)[0] > local_volumetric_limits[1]:                                    ## verifica limite local +X
        print('>>> LOCAL LIMIT +X')
        cnc_mode_switch()
        return 1
    elif get_local_current_position(local_axes,local_origin,actor4_position)[1] < local_volumetric_limits[2]:                                    ## verifica limite local -Y
        print('>>> LOCAL LIMIT -Y')
        cnc_mode_switch()
        return 1
    elif get_local_current_position(local_axes,local_origin,actor4_position)[1] > local_volumetric_limits[3]:                                    ## verifica limite local +Y
        print('>>> LOCAL LIMIT +Y')
        cnc_mode_switch()
        return 1
    elif get_local_current_position(local_axes,local_origin,actor4_position)[2] < local_volumetric_limits[4]:                                    ## verifica limite local -Z
        print('>>> LOCAL LIMIT -Z')
        cnc_mode_switch()
        return 1
    elif get_local_current_position(local_axes,local_origin,actor4_position)[2] > local_volumetric_limits[5]:                                    ## verifica limite local +Z
        print('>>> LOCAL LIMIT +Z')
        cnc_mode_switch()
        return 1

def sync_actors_movement(actor4_position,actor3_position,actor2_position,pos):

    actor4_position = pos                         #### POSIÇÃO INICIAL EM Z

    actor3_position = [actor3_position[0]           ,              ## ponte
                       actor4_position[1]      +   60.5,                           #### POSIÇÃO INICIAL EM Y
                       actor3_position[2]]


    actor2_position = [actor4_position[0]   -   150,            ## capa do z    #### POSIÇÃO INICIAL EM X
                       actor4_position[1]      +   360,
                       actor2_position[2]]

    return actor4_position, actor3_position, actor2_position

def path_from_local_to_global_coordinates(local_origin,local_axes,local_linear_path):
    print('CALCULATING GLOBAL LINEAR PATH')
    global_points = local_origin + (local_axes @ local_linear_path.T).T
    print(global_points)
    return global_points

def calculate_linear_distance(local_current_position,new_position):
    print('CALCULATING LINEAR DISTANCE')
    #distance = new_position - local_current_position
    distance = np.linalg.norm(new_position - local_current_position)
    print(float(distance))
    return float(distance)

def calculate_axis_distance(local_current_position,new_position):
    print('CALCULATING AXIS DISTANCE')
    distance = new_position - local_current_position
    return distance

def calculate_new_relative_coordinates(x,y,z,actor4_position,local_current_position):       ### função para calcular novas coordenadas em relação a posição atual (local_current_position)
    print('CALCULATING NEW LOCAL COORDINATES')
    displacement = [x,
                    y,
                    z]
    new_position = local_current_position + displacement
    print(new_position)
    return new_position

def calculate_new_local_coordinates(x,y,z,actor4_position,local_origin,local_current_position):          ### função para calcular novas coordenadas em relação ao sistema de coordenadas local (local_current_position)  
    print('CALCULATING NEW LOCAL COORDINATES')
    displacement = [x,
                    y,
                    z]
    new_position = displacement
    print(new_position)
    return new_position

def linear_path(distance,local_current_position, new_position,move_step):
    print('CALCULATING LOCAL LINEAR PATH')
    #print('LINEAR PATH INPUT ',local_current_position[2],new_position[2],distance,move_step)
    xpath = (np.linspace(local_current_position[0],new_position[0],int(distance/move_step)))
    ypath = (np.linspace(local_current_position[1],new_position[1],int(distance/move_step)))
    zpath = (np.linspace(local_current_position[2],new_position[2],int(distance/move_step)))
    print('X PATH: ',xpath,'\nY PATH: ',ypath,'\nZ PATH: ',zpath)
    
    return np.stack((xpath, ypath, zpath), axis=1)

if override_pos_limits_state == 1:
    print('>>> OVERRIDE POS LIMITS')
else:
    if check_local_volumetric_limits(local_axes,local_origin)== 1:
        print('>>> START POS NOT OK',actor4_position)
    if check_local_volumetric_limits(local_axes,local_origin) == None: 
        print('>>> START POS OK')

def create_coordinate_window():

    coord_renderer = vtk.vtkRenderer()
    coord_render_window = vtk.vtkRenderWindow()
    coord_render_window.SetWindowName("Features")
    coord_render_window.AddRenderer(coord_renderer)
    coord_render_window.SetSize(400, 600)
    coord_render_window.SetPosition(1100,0)
    
    text_actor = vtk.vtkTextActor()
    text_actor.SetInput("Registered points:\n")
    text_actor.GetTextProperty().SetFontSize(16)
    text_actor.GetTextProperty().SetColor(1, 1, 1)
    text_actor.SetPosition(0, 0)
    
    coord_renderer.AddActor(text_actor)
    coord_renderer.SetBackground(0.2, 0.2, 0.2)
    
    return coord_render_window, text_actor

coord_window, coord_text_actor = create_coordinate_window()

def update_coordinate_window(text_actor, positions):
    text = "\n\nRegistered Points:\n\n"
    for i, pos in enumerate(positions):
        text += f"Point {i+1}: ({pos[0]:.4f}, {pos[1]:.4f}, {pos[2]:.4f})\n"
    
    text_actor.SetInput(text)

    colors = vtkNamedColors()

def calculate_avg_point(p0,p1,p2):

    print(">>> CALCULATE AVERAGE POINT",p0,p1,p2)
    x = (p0[0]+p1[0]+p2[0])/3
    y = (p0[1]+p1[1]+p2[1])/3
    z = (p0[2]+p1[2]+p2[2])/3

    print(">>> AVERAGE POINT LOCATION: ", x, y, z)

    return x,y,z

""""
def create_circle(p0,p1,p2):

    polygonSource = vtkRegularPolygonSource()
    polygonSource.GeneratePolygonOff()
    polygonSource.SetNumberOfSides(50)                                           
    polygonSource.SetCenter(calculate_avg_point(p0,p1,p2))                                   
    polygonSource.SetRadius(max(calculate_linear_distance(p0,p1),
                                calculate_linear_distance(p1,p2),
                                calculate_linear_distance(p2,p0))/2)
    #create_sphere(calculate_avg_point(p0,p1,p2),8,(1,1,1))
    polygonSource.SetNormal(create_plane(p0,p1,p2,0)[1])
    polygonSource.Update()
    #print("DISTANCE P0-P1",calculate_linear_distance(p0,p1))
    #print("DISTANCE P1-P2",calculate_linear_distance(p1,p2))
    mapper = vtkPolyDataMapper()
    mapper.SetInputConnection(polygonSource.GetOutputPort())
    actor = vtkActor()
    actor.SetMapper(mapper)
    actor.GetProperty().SetColor(1,1,1)

    renderer.AddActor(actor)
    renderWindow.Render()
    return actor
"""

def create_circle(p0,p1,p2):

    polygonSource = vtkRegularPolygonSource()
    polygonSource.GeneratePolygonOff()
    polygonSource.SetNumberOfSides(360)                                           
    center = calculate_avg_point(p0,p1,p2)
    polygonSource.SetCenter(center)       
    polygonSource.SetRadius((calculate_linear_distance(p0,center)+
                            calculate_linear_distance(p1,center)+
                            calculate_linear_distance(p2,center))/3)
    #create_sphere(calculate_avg_point(p0,p1,p2),8,(1,1,1))
    polygonSource.SetNormal(create_plane(p0,p1,p2,0,0)[1])
    polygonSource.Update()
    #print("DISTANCE P0-P1",calculate_linear_distance(p0,p1))
    #print("DISTANCE P1-P2",calculate_linear_distance(p1,p2))
    mapper = vtkPolyDataMapper()
    mapper.SetInputConnection(polygonSource.GetOutputPort())
    actor = vtkActor()
    actor.SetMapper(mapper)
    actor.GetProperty().SetColor(39/255,221/255,232/255)

    poly_data = polygonSource.GetOutput()
    points = poly_data.GetPoints()

    circumference_points = []
    for i in range(points.GetNumberOfPoints()):
        point = points.GetPoint(i)
        circumference_points.append(point)

    renderer.AddActor(actor)
    renderWindow.Render()
    return actor, circumference_points

def create_axis(x,y,z,rotatex):
    
    print('>>> INSERT AXIS')
    axes2 = vtk.vtkAxesActor()
    axes2.SetTotalLength(scale*500, scale*500, scale*500)
    axes2.SetShaftTypeToCylinder()
    axes2.SetCylinderRadius(0.02)
    axes2.GetXAxisCaptionActor2D().GetTextActor().GetTextProperty().SetColor(1, 0, 0)
    axes2.GetYAxisCaptionActor2D().GetTextActor().GetTextProperty().SetColor(0, 1, 0)
    axes2.GetZAxisCaptionActor2D().GetTextActor().GetTextProperty().SetColor(0, 0, 1)

    transform = vtk.vtkTransform()
    transform.Translate(0,0,z)
    transform.RotateX(rotatex/2)
    axes2.SetUserTransform(transform)

    renderer.AddActor(axes2)
    renderWindow.Render()
    return axes2

def calculate_angle_between_vectors(v0, v1):
    
    print('\n>>> ANGLE BETWEEN VECTORS')
    v0 = np.array(v0)
    v1 = np.array(v1)
    print('VECTORS :',v0,v1)
    dot_product = np.dot(v0, v1)
    magnitudes = np.linalg.norm(v0) * np.linalg.norm(v1)
    
    if magnitudes == 0:
        print("> VECTOR'S MAGNITUDE IS ZERO")
        return 0
    
    result = np.degrees(np.arccos(np.clip(dot_product / magnitudes, -1.0, 1.0)))
    result2 = 360 - result
    print('>>> ANGLE RESULT 1: ', result)
    print('>>> ANGLE RESULT 2: ', result2)
    
    return result, result2

def create_vector(p0, p1, opacity):

    print('\n>>> CREATE VECTOR')
    print('p0: ', p0)
    print('p1: ', p1)

    x = p0[0]-p1[0]
    y = p0[1]-p1[1]
    z = p0[2]-p1[2]

    direction = [x, y, z]
    length = (x**2 + y**2 + z**2)**0.5
    
    if length > 0:
        direction = [x/length, y/length, z/length]

    lineSource = vtk.vtkLineSource()
    lineSource.SetPoint1(p0)
    lineSource.SetPoint2(p1)
    lineMapper = vtk.vtkPolyDataMapper()
    lineMapper.SetInputConnection(lineSource.GetOutputPort())
    lineActor = vtk.vtkActor()
    lineActor.SetMapper(lineMapper)
    lineActor.GetProperty().SetColor(1,1,1)
    lineActor.GetProperty().SetOpacity(opacity)
    lineActor.GetProperty().SetLineWidth(1)
    renderer.AddActor(lineActor)
    
    coneSource2 = vtk.vtkConeSource()
    coneSource2.SetCenter(p0)
    coneSource2.SetDirection([d for d in direction]) 
    coneSource2.SetHeight(18)
    coneSource2.SetRadius(6)
    
    coneMapper2 = vtk.vtkPolyDataMapper()
    coneMapper2.SetInputConnection(coneSource2.GetOutputPort())
    coneActor2 = vtk.vtkActor()
    coneActor2.SetMapper(coneMapper2)
    coneActor2.GetProperty().SetColor(0, 0, 0)
    coneActor2.GetProperty().SetOpacity(opacity)
    renderer.AddActor(coneActor2)

    renderWindow.Render()

    return x, y, z

def create_plane(p0, p1, p2, opacity, sceneNormalactor):
    print('\n>>> CREATE PLANE')
    planeSource = vtk.vtkPlaneSource()
    planeSource.SetCenter(p1)
    planeSource.SetPoint1(p0)
    planeSource.SetPoint2(p2)

    v1 = create_vector(p0,p1,0)
    #print(v1)
    v2 = create_vector(p0,p2,0)

    mapper = vtk.vtkPolyDataMapper() 
    mapper.SetInputConnection(planeSource.GetOutputPort())
    actor = vtkActor()
    actor.SetMapper(mapper)
    #actor.GetProperty().SetColor(39/255,221/255,232/255)
    actor.GetProperty().SetOpacity(0)
    
    wireframeMapper = vtk.vtkPolyDataMapper()
    wireframeMapper.SetInputConnection(planeSource.GetOutputPort())
    wireframeActor = vtkActor()
    wireframeActor.SetMapper(wireframeMapper)
    wireframeActor.GetProperty().SetRepresentationToWireframe()
    wireframeActor.GetProperty().SetColor(39/255,221/255,232/255) 
    wireframeActor.GetProperty().SetLineWidth(2)
    wireframeActor.GetProperty().SetLighting(False)
                
    center = calculate_avg_point(p0,p1,p2)     
    #create_sphere(center,2,(1,1,1))
    
    planeNormal = planeSource.GetNormal()    
    planeSource.Update()

    if sceneNormalactor == 1:

        featureEdges = vtkFeatureEdges()
        featureEdges.SetInputConnection(planeSource.GetOutputPort())
        featureEdges.BoundaryEdgesOn()
        featureEdges.FeatureEdgesOff()
        featureEdges.ManifoldEdgesOff()
        featureEdges.NonManifoldEdgesOff()

        edgeMapper = vtk.vtkPolyDataMapper()
        edgeMapper.SetInputConnection(featureEdges.GetOutputPort())

        edgeActor = vtk.vtkActor()
        edgeActor.SetMapper(edgeMapper)
        edgeActor.GetProperty().SetColor(1,1,1)
        edgeActor.GetProperty().SetLineWidth(0.5)

        end_point = [
            center[0] + planeNormal[0] * 50,
            center[1] + planeNormal[1] * 50, 
            center[2] + planeNormal[2] * 50
        ]
        
        u = create_vector(end_point,center,1)
        a_z = calculate_angle_between_vectors((u[0],u[1],u[2]),(v1[0],v1[1],v1[2]))
        #create_axis(center[0],center[1],center[2],a_z[0])                                   #### A_Z REPRESENTA O ÂNGULO INTERNO OBTIDO DE CALCULATE_ANGLE_BETWEEN_VECTORS
        renderer.AddActor(edgeActor)

    renderer.AddActor(actor)
    renderer.AddActor(wireframeActor)
    print(">>> PLANE NORMAL:", planeNormal)
    renderWindow.Render()
    return actor, planeNormal, planeSource, v1, center[2]

def create_sphere(center, radius, color):
    sphereSource = vtk.vtkSphereSource()
    sphereSource.SetCenter(center)
    sphereSource.SetRadius(radius)
    sphereSource.SetPhiResolution(100)
    sphereSource.SetThetaResolution(100)
    mapper = vtk.vtkPolyDataMapper()
    mapper.SetInputConnection(sphereSource.GetOutputPort())
    sphere_actor = vtk.vtkActor()
    sphere_actor.SetMapper(mapper)
    sphere_actor.GetProperty().SetColor(*color)
    sphere_actor.GetProperty().SetOpacity(1)
    sphere_actor.GetProperty().SetEdgeVisibility(False)
    renderer.AddActor(sphere_actor)
    renderWindow.Render()
    
    return sphere_actor

def create_volume_box_actor(limits, color):
    x_min, x_max, y_min, y_max, z_min, z_max = limits
    center = [(x_max + x_min) / 2, (y_max + y_min) / 2, (z_max + z_min) / 2]
    x_length = x_max - x_min
    y_length = y_max - y_min
    z_length = z_max - z_min

    cube = vtk.vtkCubeSource()
    cube.SetCenter(*center)
    cube.SetXLength(x_length)
    cube.SetYLength(y_length)
    cube.SetZLength(z_length)

    mapper = vtk.vtkPolyDataMapper()
    mapper.SetInputConnection(cube.GetOutputPort())

    volumeactor = vtk.vtkActor()
    volumeactor.SetMapper(mapper)
    volumeactor.GetProperty().SetColor(*color)
    volumeactor.GetProperty().SetOpacity(0.2)
    volumeactor.GetProperty().SetEdgeVisibility(True)
    volumeactor.GetProperty().SetEdgeColor(0.5, 0.5, 0.5)
    volumeactor.GetProperty().SetLineWidth(1.5)

    return volumeactor

global_volume_actor = create_volume_box_actor(global_volumetric_limits,color=(0.5, 0, 0))
local_volume_actor = create_volume_box_actor(local_volumetric_limits,color=(0,0.5, 0))
#local_volume_actor = create_volume_box_actor(local_volumetric_limits,color=(0.5, 0.5, 0))
#safe_volume_actor = create_volume_box_actor(local_safe_limits,color=(0, 0.5, 0))

def translate_in_volume(actor4_position,actor3_position,actor2_position,x,y,z):      ## o translate é uma função para ir de ponto A[x,y,z] a B[x1,y1,z1], onde A é a localização atual dentro do volume
    #print('KEY L')              
    local_current_position = get_local_current_position(local_axes,local_origin,actor4_position)
    new_position = calculate_new_local_coordinates(x,
                                                   y,
                                                   z,
                                                    actor4_position,local_origin,local_current_position)
    local_linear_path = linear_path(calculate_linear_distance(local_current_position,new_position),local_current_position,new_position,1)
    global_linear_path = path_from_local_to_global_coordinates(local_origin,local_axes,local_linear_path)
    sleep = 1 / speed
    cnc_mode_switch()
    occurence = 0
    for pos in global_linear_path:
        if check_local_volumetric_limits(local_axes,local_origin)== 1 or cnc_mode_state == False:
            occurence = 1
            break
        actor4.SetPosition(*(sync_actors_movement(actor4_position,actor3_position,actor2_position,pos)[0]))
        actor3.SetPosition(*(sync_actors_movement(actor4_position,actor3_position,actor2_position,pos)[1]))
        actor2.SetPosition(*(sync_actors_movement(actor4_position,actor3_position,actor2_position,pos)[2]))
        actor4_position = sync_actors_movement(actor4_position,actor3_position,actor2_position,pos)[0]
        actor3_position = sync_actors_movement(actor4_position,actor3_position,actor2_position,pos)[1]
        actor2_position = sync_actors_movement(actor4_position,actor3_position,actor2_position,pos)[2]
        renderWindow.Render()
        t.sleep(sleep)
    
    if occurence == 0:
        cnc_mode_switch()
    
    return actor4_position,actor3_position,actor2_position

def create_3dline(p0, p1):
    linesource = vtk.vtkLineSource()
    linesource.SetPoint1(p0)
    linesource.SetPoint2(p1)
    linesource.Update()
    mapper = vtk.vtkPolyDataMapper()
    mapper.SetInputConnection(linesource.GetOutputPort())
    actor = vtk.vtkActor()
    actor.SetMapper(mapper)
    actor.GetProperty().SetColor(39/255,221/255,232/255)
    actor.GetProperty().SetLineWidth(2)
    lineoutput = linesource.GetOutput()

    renderer.AddActor(actor)
    renderWindow.Render()
    
    return actor, lineoutput

def keypress_callback(obj, event):
        global actor2_position, actor3_position, actor4_position, actor5_position, local_volumetric_limits, global_volumetric_limits
        #print(user_axis_control)
        if user_axis_control_check() == False:
            print('USER AXIS CONTROL', user_axis_control)
            e_sftlck_state_lock(actor4_position)
            pass
        elif user_axis_control_check() == True:
            key = obj.GetKeySym()

            ## actor1 navigatorbase
            ## actor2 navigator-z 
            ## actor3 navigator-y
            ## actor4 navigator-z2

            if key == 'Left':                                               ## MOVER -X

                if override_pos_limits_state == 1:
                    actor4_position[0] -= move_step                         ## avanço
                    actor2_position[0] -= move_step
                    print('KEY LEFT global_actor4_position ', actor4_position, '/ local_current_position ', get_local_current_position(local_axes,local_origin,actor4_position))

                else:
                    if check_local_volumetric_limits(local_axes,local_origin) == None:     ## segue o baile
                        actor4_position[0] -= move_step                             ## avanço
                        actor2_position[0] -= move_step                             ## avanço
                        print('KEY LEFT global_actor4_position ', actor4_position, '/ local_current_position ', get_local_current_position(local_axes,local_origin,actor4_position))

                    if check_local_volumetric_limits(local_axes,local_origin) == 1:        ## chegou no limite
                        check_global_volumetric_limits(actor4_position)
                        actor4_position[0] -= move_step                             ## avanço
                        actor2_position[0] -= move_step
                
            elif key == 'Right':                                            ## MOVER +X
                

                if override_pos_limits_state == 1:
                    actor4_position[0] += move_step                         ## avanço
                    actor2_position[0] += move_step
                    print('KEY RIGHT global_actor4_position ', actor4_position, '/ local_current_position ', get_local_current_position(local_axes,local_origin,actor4_position))

                else:
                    if check_local_volumetric_limits(local_axes,local_origin) == None:
                        actor4_position[0] += move_step
                        actor2_position[0] += move_step
                        print('KEY RIGHT global_actor4_position ', actor4_position, '/ local_current_position ', get_local_current_position(local_axes,local_origin,actor4_position))

                    if check_local_volumetric_limits(local_axes,local_origin) == 1:
                        check_global_volumetric_limits(actor4_position)
                        actor4_position[0] += move_step
                        actor2_position[0] += move_step

            elif key == 'Up':                                               ## MOVER +Y
                
                if override_pos_limits_state == 1:
                    actor4_position[1] += move_step                         ## avanço
                    actor3_position[1] += move_step
                    actor2_position[1] += move_step
                    print('KEY UP global_actor4_position ', actor4_position, '/ local_current_position ', get_local_current_position(local_axes,local_origin,actor4_position))

                else:

                    if check_local_volumetric_limits(local_axes,local_origin) == None:    
                        actor4_position[1] += move_step
                        actor3_position[1] += move_step
                        actor2_position[1] += move_step
                        print('KEY UP global_actor4_position ', actor4_position, '/ local_current_position ', get_local_current_position(local_axes,local_origin,actor4_position))

                    if check_local_volumetric_limits(local_axes,local_origin) == 1:
                        check_global_volumetric_limits(actor4_position)
                        actor4_position[1] += move_step
                        actor3_position[1] += move_step
                        actor2_position[1] += move_step
                        
            elif key == 'Down':                                             ## MOVER -Y
                
                if override_pos_limits_state == 1:
                    actor4_position[1] -= move_step                         ## avanço
                    actor3_position[1] -= move_step
                    actor2_position[1] -= move_step
                    print('KEY DOWN global_actor4_position ', actor4_position, '/ local_current_position ', get_local_current_position(local_axes,local_origin,actor4_position))

                else:
                
                    if check_local_volumetric_limits(local_axes,local_origin) == None:    ## segue o baile
                        actor4_position[1] -= move_step                     ## avanço
                        actor3_position[1] -= move_step                     ## avanço
                        actor2_position[1] -= move_step                     ## avanço
                        print('KEY DOWN global_actor4_position ', actor4_position, '/ local_current_position ', get_local_current_position(local_axes,local_origin,actor4_position))

                    if check_local_volumetric_limits(local_axes,local_origin) == 1:       ## chegou no limite
                        check_global_volumetric_limits(actor4_position)
                        actor4_position[1] -= move_step                     ## reação ao avanço
                        actor3_position[1] -= move_step                     ## reação ao avanço
                        actor2_position[1] -= move_step                     ## reação ao avanço
                        #actor4_position[1] -= move_step                     ## reação ao avanço
                        #actor3_position[1] -= move_step                     ## reação ao avanço
                        #actor2_position[1] -= move_step                     ## reação ao avanço

            elif key == 'm':                                                ## MOVER -Z
                
                if override_pos_limits_state == 1:
                    actor4_position[2] -= move_step
                    print('KEY m global_actor4_position ', actor4_position, '/ local_current_position ', get_local_current_position(local_axes,local_origin,actor4_position))

                else:
                    if check_local_volumetric_limits(local_axes,local_origin) == None:
                        actor4_position[2] -= move_step
                        print('KEY m global_actor4_position ', actor4_position, '/ local_current_position ', get_local_current_position(local_axes,local_origin,actor4_position))
                    if check_local_volumetric_limits(local_axes,local_origin) == 1:
                        check_global_volumetric_limits(actor4_position)
                        actor4_position[2] -= move_step

            elif key == 'k':                                                ## MOVER +Z
                                
                if override_pos_limits_state == 1:
                    actor4_position[2] += move_step
                    print('KEY k global_actor4_position ', actor4_position, '/ local_current_position ', get_local_current_position(local_axes,local_origin,actor4_position))

                else:
                    if check_local_volumetric_limits(local_axes,local_origin) == None:
                        actor4_position[2] += move_step
                        print('KEY k global_actor4_position ', actor4_position, '/ local_current_position ', get_local_current_position(local_axes,local_origin,actor4_position))
                    if check_local_volumetric_limits(local_axes,local_origin) == 1:
                        check_global_volumetric_limits(actor4_position)
                        actor4_position[2] += move_step              

            elif key == '3':                                             ## INPUT COMMAND
                print("key ",key)
                print(">>> BASE SYS COORDINATE TRANSFORM: ")

                vplane = create_plane(path_from_local_to_global_coordinates(local_origin,local_axes,cmm_position[-1]),
                                path_from_local_to_global_coordinates(local_origin,local_axes,cmm_position[-2]),
                                path_from_local_to_global_coordinates(local_origin,local_axes,cmm_position[-3]),
                                0.5,
                                0)[2]
                
                line = create_3dline(path_from_local_to_global_coordinates(local_origin,local_axes,cmm_position[-4]),
                              path_from_local_to_global_coordinates(local_origin,local_axes,cmm_position[-5]))[1]

                cutter = vtk.vtkCutter()
                cutter.SetCutFunction(vtk.vtkPlane())
                cutter.SetInputData(line)
                cutter.Update()
                cutterout = cutter.GetOutput()
                print('GetNumberOfPoints: ',cutterout.GetNumberOfPoints())
                print(cutterout.GetPoint(0))

                renderWindow.Render()

            elif key == '2':                                             ## INPUT COMMAND
                print("key ",key)
                command = input("> NAVIGATOR: ")

                if command == 'SPHERE' or command == 'sphere':        ## TRANSLATE TO SPHERE
                    
                    print('\nCOMMAND:',command)
                    local_current_position = get_local_current_position(local_axes,local_origin,actor4_position)
                    translate = translate_in_volume(actor4_position,
                                                    actor3_position,
                                                    actor2_position,
                                                    get_local_current_position(local_axes,local_origin,actor5_position)[0],
                                                    get_local_current_position(local_axes,local_origin,actor5_position)[1],
                                                    get_local_current_position(local_axes,local_origin,actor5_position)[2]  +   20)
                    
                    actor4_position = translate[0]
                    actor3_position = translate[1]
                    actor2_position = translate[2]

                if command == 'MOVETO' or command == 'moveto':

                    print('\nCOMMAND:',command)
                    
                    local_current_position = get_local_current_position(local_axes,local_origin,actor4_position)
                    translate = translate_in_volume(actor4_position,
                                                    actor3_position,
                                                    actor2_position,
                                                    float(input('x:')),
                                                    float(input('y:')),
                                                    float(input('z:')))
                    
                    actor4_position = translate[0]
                    actor3_position = translate[1]
                    actor2_position = translate[2]

                if command == 'CIRCLE' or command == 'circle':
                    print('\nKEY ',key)
                    print('>>> CREATE CIRCLE')

                    if check_cnc_position_list(cmm_position,2) == 0:
                        print('>>> NOT ENOUGH ELEMENTS AVAILABLE')
                    else:
                        
                        circle = create_circle(path_from_local_to_global_coordinates(local_origin,local_axes,cmm_position[-1]),
                                    path_from_local_to_global_coordinates(local_origin,local_axes,cmm_position[-2]),
                                    path_from_local_to_global_coordinates(local_origin,local_axes,cmm_position[-3]))
                    
                    
                if command == 'CIRCLECNC' or command == 'circlecnc':
                    print('\nKEY ',key)
                    print('>>> CREATE CIRCLE')

                    if check_cnc_position_list(cmm_position,2) == 0:
                        print('>>> NOT ENOUGH ELEMENTS AVAILABLE')
                    else:
                        
                        circle = create_circle(path_from_local_to_global_coordinates(local_origin,local_axes,cmm_position[-1]),
                                    path_from_local_to_global_coordinates(local_origin,local_axes,cmm_position[-2]),
                                    path_from_local_to_global_coordinates(local_origin,local_axes,cmm_position[-3]))
                    
                        local_current_position = get_local_current_position(local_axes,local_origin,actor4_position)
                        
                        for i in circle[1]:
                            translate = translate_in_volume(actor4_position,
                                                            actor3_position,
                                                            actor2_position,
                                                            (get_local_current_position(local_axes,local_origin,i[0])[0]),
                                                            (get_local_current_position(local_axes,local_origin,i[1])[1]),
                                                            (get_local_current_position(local_axes,local_origin,i[2])[2]))
                            actor4_position = translate[0]
                            actor3_position = translate[1]
                            actor2_position = translate[2]
                            renderWindow.Render()

                            #t.sleep(0.01)
                                        
                        actor4_position = translate[0]
                        actor3_position = translate[1]
                        actor2_position = translate[2]

                if command == 'RECALL' or command == 'recall':
                        local_current_position = get_local_current_position(local_axes,local_origin,actor4_position)
                        
                        for i in cmm_position:
                            translate = translate_in_volume(actor4_position,
                                                            actor3_position,
                                                            actor2_position,
                                                            (i)[0],
                                                            (i)[1],
                                                            (i)[2])

                            actor4_position = translate[0]
                            actor3_position = translate[1]
                            actor2_position = translate[2]
                            renderWindow.Render()
    
                        actor4_position = translate[0]
                        actor3_position = translate[1]
                        actor2_position = translate[2]
                else:
                    print(">>> UNKNOWN COMMAND")
                    print(">>> EXIT COMMAND INPUT MODE")

            elif key == 'O':        ## GO TO CENTER VOLUMETRIC LIMITS

                print('\nKEY 0')
                local_current_position = get_local_current_position(local_axes,local_origin,actor4_position)
                translate = translate_in_volume(actor4_position,
                                                actor3_position,
                                                actor2_position,
                                                local_volumetric_limits[1]/2,
                                                local_volumetric_limits[2]/2,
                                                local_volumetric_limits[4]/2,)
                
                actor4_position = translate[0]
                actor3_position = translate[1]
                actor2_position = translate[2]
            
            
    
            elif key == 'L':        ## CREATE 3D LINE
                print('\nKEY L')
                print('>>> CREATE 3DLINE')

                if check_cnc_position_list(cmm_position,1) == 0:
                    print('>>> NOT ENOUGH ELEMENTS AVAILABLE')
                else:
            
                    print('>>> STARTPOINT: ', path_from_local_to_global_coordinates(local_origin,local_axes,cmm_position[-1]))
                    print('>>> ENDPOINT: ', path_from_local_to_global_coordinates(local_origin,local_axes,cmm_position[-2]))

                    create_3dline(path_from_local_to_global_coordinates(local_origin,local_axes,cmm_position[-1]),
                                path_from_local_to_global_coordinates(local_origin,local_axes,cmm_position[-2]))
                
            elif key == 'H':        ## HOMING FUNCTION
                print('\nKEY H')
                local_current_position = get_local_current_position(local_axes,local_origin,actor4_position)
                new_position = calculate_new_local_coordinates(local_current_position[0],local_current_position[1],0,actor4_position,local_origin,local_current_position)
                local_linear_path = linear_path(calculate_linear_distance(local_current_position,new_position),local_current_position,new_position,1)
                global_linear_path = path_from_local_to_global_coordinates(local_origin,local_axes,local_linear_path)
                sleep = 1 / speed
                cnc_mode_switch()
                occurence = 0
                for pos in global_linear_path:
                    if check_local_volumetric_limits(local_axes,local_origin)== 1 or cnc_mode_state == False:
                        occurence = 1
                        break
                    actor4.SetPosition(*(sync_actors_movement(actor4_position,actor3_position,actor2_position,pos)[0]))
                    actor3.SetPosition(*(sync_actors_movement(actor4_position,actor3_position,actor2_position,pos)[1]))
                    actor2.SetPosition(*(sync_actors_movement(actor4_position,actor3_position,actor2_position,pos)[2]))
                    actor4_position = sync_actors_movement(actor4_position,actor3_position,actor2_position,pos)[0]
                    actor3_position = sync_actors_movement(actor4_position,actor3_position,actor2_position,pos)[1]
                    actor2_position = sync_actors_movement(actor4_position,actor3_position,actor2_position,pos)[2]
                    renderWindow.Render()
                    t.sleep(sleep)

                new_position2 = calculate_new_local_coordinates(0,0,0,actor4_position,local_origin,local_current_position)
                local_current_position2 = get_local_current_position(local_axes,local_origin,actor4_position)
                local_linear_path2 = linear_path(calculate_linear_distance(local_current_position2,new_position2),local_current_position2,new_position2,1)
                global_linear_path2 = path_from_local_to_global_coordinates(local_origin,local_axes,local_linear_path2)
                for pos in global_linear_path2:
                    if check_local_volumetric_limits(local_axes,local_origin) == 1 or cnc_mode_state == False:
                        occurence = 1
                        break
                    actor4.SetPosition(*(sync_actors_movement(actor4_position,actor3_position,actor2_position,pos)[0]))
                    actor3.SetPosition(*(sync_actors_movement(actor4_position,actor3_position,actor2_position,pos)[1]))
                    actor2.SetPosition(*(sync_actors_movement(actor4_position,actor3_position,actor2_position,pos)[2]))
                    actor4_position = sync_actors_movement(actor4_position,actor3_position,actor2_position,pos)[0]
                    actor3_position = sync_actors_movement(actor4_position,actor3_position,actor2_position,pos)[1]
                    actor2_position = sync_actors_movement(actor4_position,actor3_position,actor2_position,pos)[2]
                    renderWindow.Render()
                    t.sleep(sleep)

                new_position3 = calculate_new_local_coordinates(10,-10,-10,actor4_position,local_origin,local_current_position)
                local_current_position3 = get_local_current_position(local_axes,local_origin,actor4_position)
                local_linear_path3 = linear_path(calculate_linear_distance(local_current_position3,new_position3),local_current_position3,new_position3,1)
                global_linear_path3 = path_from_local_to_global_coordinates(local_origin,local_axes,local_linear_path3)
                for pos in global_linear_path3:
                    if check_local_volumetric_limits(local_axes,local_origin)== 1 or cnc_mode_state == False:
                        occurence = 1
                        break
                    actor4.SetPosition(*(sync_actors_movement(actor4_position,actor3_position,actor2_position,pos)[0]))
                    actor3.SetPosition(*(sync_actors_movement(actor4_position,actor3_position,actor2_position,pos)[1]))
                    actor2.SetPosition(*(sync_actors_movement(actor4_position,actor3_position,actor2_position,pos)[2]))
                    actor4_position = sync_actors_movement(actor4_position,actor3_position,actor2_position,pos)[0]
                    actor3_position = sync_actors_movement(actor4_position,actor3_position,actor2_position,pos)[1]
                    actor2_position = sync_actors_movement(actor4_position,actor3_position,actor2_position,pos)[2]
                    renderWindow.Render()
                    t.sleep(sleep)
                if occurence == 0:
                    cnc_mode_switch()


            elif key == 'h':        ## LOCAL GO TO
                print('KEY h')
                local_current_position = get_local_current_position(local_axes,local_origin,actor4_position)
                new_position = calculate_new_local_coordinates(float(input('x:')),float(input('y:')),float(input('z:')),actor4_position,local_origin,local_current_position)
                distance = calculate_linear_distance(local_current_position,new_position)
                local_linear_path = linear_path(distance,local_current_position,new_position,move_step)
                global_linear_path = path_from_local_to_global_coordinates(local_origin,local_axes,local_linear_path)
                sleep = move_step / speed
                cnc_mode_switch()
                occurence = 0
                for pos in global_linear_path:
                    if check_local_volumetric_limits(local_axes,local_origin)== 1 or cnc_mode_state == False:
                        occurence = 1
                        break

                    actor4.SetPosition(*(sync_actors_movement(actor4_position,actor3_position,actor2_position,pos)[0]))
                    actor3.SetPosition(*(sync_actors_movement(actor4_position,actor3_position,actor2_position,pos)[1]))
                    actor2.SetPosition(*(sync_actors_movement(actor4_position,actor3_position,actor2_position,pos)[2]))
                    actor4_position = sync_actors_movement(actor4_position,actor3_position,actor2_position,pos)[0]
                    actor3_position = sync_actors_movement(actor4_position,actor3_position,actor2_position,pos)[1]
                    actor2_position = sync_actors_movement(actor4_position,actor3_position,actor2_position,pos)[2]

                    renderWindow.Render()
                    t.sleep(sleep)
                if occurence == 0:
                    cnc_mode_switch()
                
            elif key == 'g':        ## TRANSLATE TESTS
                print('\nKEY ',key)
                local_current_position = get_local_current_position(local_axes,local_origin,actor4_position)
                new_position = calculate_new_relative_coordinates(250,-650,-180,actor4_position,local_current_position)
                distance = calculate_linear_distance(local_current_position,new_position)
                local_linear_path = linear_path(distance,local_current_position,new_position,move_step)
                global_linear_path = path_from_local_to_global_coordinates(local_origin,local_axes,local_linear_path)
                sleep = move_step / speed
                cnc_mode_switch()
                occurence = 0
                for pos in global_linear_path:
                    if check_local_volumetric_limits(local_axes,local_origin)== 1 or cnc_mode_state == False:
                        occurence = 1
                        break

                    actor4.SetPosition(*(sync_actors_movement(actor4_position,actor3_position,actor2_position,pos)[0]))
                    actor3.SetPosition(*(sync_actors_movement(actor4_position,actor3_position,actor2_position,pos)[1]))
                    actor2.SetPosition(*(sync_actors_movement(actor4_position,actor3_position,actor2_position,pos)[2]))
                    actor4_position = sync_actors_movement(actor4_position,actor3_position,actor2_position,pos)[0]
                    actor3_position = sync_actors_movement(actor4_position,actor3_position,actor2_position,pos)[1]
                    actor2_position = sync_actors_movement(actor4_position,actor3_position,actor2_position,pos)[2]

                    renderWindow.Render()
                    t.sleep(sleep)
                if occurence == 0:
                    cnc_mode_switch()                  

            elif key == '9':
                
                print(f'\nKEY {key}')
                print('>>> CREATE AXIS')

                if check_cnc_position_list(cmm_position,3) == 0:
                    print('>>> NOT ENOUGH ELEMENTS AVAILABLE')
                else:
            
                    create_axis(0,
                                0,
                                create_plane(path_from_local_to_global_coordinates(local_origin,local_axes,cmm_position[-1]),
                                    path_from_local_to_global_coordinates(local_origin,local_axes,cmm_position[-2]),
                                    path_from_local_to_global_coordinates(local_origin,local_axes,cmm_position[-3]),
                                    0.5,
                                    1)[4],
                                1)
                    renderWindow.Render()

            elif key == '6':
                calculate_angle_between_vectors(create_vector(path_from_local_to_global_coordinates(local_origin,local_axes,cmm_position[-3]),
                                                path_from_local_to_global_coordinates(local_origin,local_axes,cmm_position[-4]),
                                                1),
                                                create_vector(path_from_local_to_global_coordinates(local_origin,local_axes,cmm_position[-1]),
                                                path_from_local_to_global_coordinates(local_origin,local_axes,cmm_position[-2]),
                                                1))
                #create_vector(path_from_local_to_global_coordinates(local_origin,local_axes,cmm_position[-1]), path_from_local_to_global_coordinates(local_origin,local_axes,cmm_position[-2]),1)
            
            elif key == '8':
                print('\nKEY ',key)
                print('>>> CREATE PLANE')

                if check_cnc_position_list(cmm_position,2) == 0:
                    print('>>> NOT ENOUGH ELEMENTS AVAILABLE')
                else:
                    create_plane(path_from_local_to_global_coordinates(local_origin,local_axes,cmm_position[-1]),
                                path_from_local_to_global_coordinates(local_origin,local_axes,cmm_position[-2]),
                                path_from_local_to_global_coordinates(local_origin,local_axes,cmm_position[-3]),
                                0.5,
                                1)
                
            elif key == '0':
                print('\nKEY ',key)
                print('STATE global_actor4_position ', actor4_position, '/ local_current_position ', get_local_current_position(local_axes,local_origin,actor4_position))
                print('STATE CMM VOLUME AVAILABLE: (',
                      (round(abs(local_volumetric_limits[1]/100),0)*100), ',',
                      (round(abs(local_volumetric_limits[2]/100),0)*100), ',',
                      (round(abs(local_volumetric_limits[4]/100),0)*100), ')',
                      )
                
                print('STATE LOCAL VOLUMETRIC LIMITS: ', local_volumetric_limits)
                print('STATE LOCAL SYS ORIGIN: ', local_origin)
                print('STATE CMM POSITION LIST: ')
                
                if len(cmm_position) == 0:
                    print('STATE NOT ENOUGH ELEMENTS AVAILABLE')
                else:
                    ki = 1
                    for k in cmm_position:
                        print(f'{ki}: {k}')
                        ki+=1
                
            elif key == '1':           ## REGISTER CMM POSITION
                print('\nKEY ',key)
                print('>>> REGISTER CMM POSITION')
                print('get_local_current_position: ',get_local_current_position(local_axes,local_origin,actor4_position))
                
                renderer.AddActor(create_sphere((actor4_position[0],
                                                 actor4_position[1],
                                                 actor4_position[2]),
                                                 4,
                                                 (0,0,0.75)))
                cmm_position.append(get_local_current_position(local_axes,local_origin,actor4_position))
                update_coordinate_window(coord_text_actor, cmm_position)
                coord_window.Render()
            
            elif key == '5':           ## CREATE CENTER SPHERE
                print('\nKEY ',key)
                print('>>> CREATE CENTER SPHERE')
                
                renderer.AddActor(create_sphere(actor5_position,
                                                25,
                                                (0.92,0.91,0.86)))

            elif key == 'V':
                print('\nKEY ',key)
                print('STATE TOGGLE SHOW LOCAL VOLUMETRIC LIMITS')
                global show_volume_bounds
                show_volume_bounds = not show_volume_bounds
                if show_volume_bounds:
                    renderer.AddActor(global_volume_actor)
                    renderer.AddActor(local_volume_actor)
                else:
                    renderer.RemoveActor(global_volume_actor)
                    renderer.RemoveActor(local_volume_actor)
                renderWindow.Render()

            elif key == 's':
                print('\nKEY ',key)
                print('>>> NO FUNCTION')

            actor2.SetPosition(*actor2_position)
            actor3.SetPosition(*actor3_position)
            actor4.SetPosition(*actor4_position)
            #actor5.SetPosition(*actor5_position)
            renderWindow.Render()

def main():
    global actor2, actor3, actor4, actor5, renderWindow, renderer

    reader1 = vtk.vtkSTLReader()
    reader1.SetFileName(r'navigatorbase3.stl')
    #reader1.SetFileName(r'F:\compsi\VTK\navigatorbase3.stl')
    reader1.Update()

    mapper1 = vtk.vtkPolyDataMapper()
    mapper1.SetInputConnection(reader1.GetOutputPort())

    actor1 = vtk.vtkActor()
    actor1.SetMapper(mapper1)
    actor1.SetPosition(*actor1_position)

    reader2 = vtk.vtkSTLReader()
    
    reader2.SetFileName(r'navigator-z.stl')
    #reader2.SetFileName(r'F:\compsi\VTK\navigator-z.stl')
    reader2.Update()

    mapper2 = vtk.vtkPolyDataMapper()
    mapper2.SetInputConnection(reader2.GetOutputPort())

    actor2 = vtk.vtkActor()
    actor2.SetMapper(mapper2)
    actor2.SetPosition(*actor2_position)

    reader3 = vtk.vtkSTLReader()
    reader3.SetFileName(r'navigator-y.stl')
    reader3.Update()
    
    mapper3 = vtk.vtkPolyDataMapper()
    mapper3.SetInputConnection(reader3.GetOutputPort())

    actor3 = vtk.vtkActor()
    actor3.SetMapper(mapper3)
    actor3.SetPosition(*actor3_position)

    reader4 = vtk.vtkSTLReader()
    
    reader4.SetFileName(r'navigator-z2.stl')
    reader4.Update()
    
    mapper4 = vtk.vtkPolyDataMapper()
    mapper4.SetInputConnection(reader4.GetOutputPort())

    actor4 = vtk.vtkActor()
    actor4.SetMapper(mapper4)
    actor4.SetPosition(*actor4_position)

    reader5 = vtk.vtkSTLReader()
    
    reader5.SetFileName(r'navigator-y.stl')
    reader5.Update()
    
    mapper5 = vtk.vtkPolyDataMapper()
    mapper5.SetInputConnection(reader5.GetOutputPort())

    actor5 = vtk.vtkActor()
    actor5.SetMapper(mapper5)
    actor5.SetPosition(*actor5_position)
    
    renderer = vtk.vtkRenderer()
    renderWindow = vtk.vtkRenderWindow()
    renderWindow.AddRenderer(renderer)
    global renderWindowInteractor_global
    renderWindowInteractor = vtk.vtkRenderWindowInteractor()
    
    renderWindowInteractor_global = renderWindowInteractor
    renderWindowInteractor.SetRenderWindow(renderWindow)

    renderer.SetOcclusionRatio(0.004)
    
    renderWindow.SetSize(1080,720)

    renderer.AddActor(actor1)
    renderer.AddActor(actor2)
    renderer.AddActor(actor3)
    renderer.AddActor(actor4)
    #renderer.AddActor(actor5)
      
    colors = vtk.vtkNamedColors()

    renderer.GradientBackgroundOn()
    
    color_bottom = [204/255.0, 204/255.0, 204/255.0]
    color_top = [59/255.0, 92/255.0, 139/255.0]     
    renderer.SetBackground(*color_bottom)
    renderer.SetBackground2(*color_top)

    actor3.SetScale(scale,scale,scale)
    actor1.SetScale(scale,scale,scale)
    actor1.RotateZ(90)
    actor4.SetScale(scale,scale,scale)
    actor5.SetScale(1000,1000,1000)
    actor2.SetScale(scale,scale,scale)
    actor2.RotateZ(90)
    actor5.RotateZ(135)

    axes = vtk.vtkAxesActor()
    axes.SetTotalLength(scale*100,scale*100,scale*100) # tamanho dos eixos
    axes.SetShaftTypeToCylinder()
    axes.SetCylinderRadius(0.02)
    axes.GetXAxisCaptionActor2D().GetTextActor().GetTextProperty().SetColor(1, 0, 0)  # X vermelho
    axes.GetYAxisCaptionActor2D().GetTextActor().GetTextProperty().SetColor(0, 1, 0)  # Y verde
    axes.GetZAxisCaptionActor2D().GetTextActor().GetTextProperty().SetColor(0, 0, 1)  # Z azul
    axes.SetPosition(0, 0, 0)  # origem

    renderer.AddActor(axes)

    camera = vtk.vtkCamera()
    camera.SetViewUp(1,1,1)  
    camera.SetPosition(-2000,-2500,1500) ## perfil bonito      
    camera.SetFocalPoint(1500, 1500, 500)
    renderer.SetActiveCamera(camera)

    global keypress_observer_id
    keypress_id = renderWindowInteractor.AddObserver("KeyPressEvent", keypress_callback)
    keypress_observer_id = keypress_id

    renderWindow.Render()
    renderWindowInteractor.Start()

if __name__ == '__main__':
    main()
