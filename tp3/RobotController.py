from coppeliasim_zmqremoteapi_client import *

class RobotController:
    def __init__(self, robot_name):
        """
            Initializes a RobotController object.

            Parameters:
            - robot_name (str): The name of the robot.

            Attributes:
            - client: The RemoteAPIClient object.
            - sim: The simulation object.
            - robot: The robot object.
            - laser: The laser object.
            - robot_name (str): The name of the robot.
            - robot_handle: The handle of the robot object.
            - left_wheel: The handle of the left wheel object.
            - right_wheel: The handle of the right wheel object.
            - goal_handle: The handle of the goal object.
            - path (list): A list to store the robot's path.
            - laser_points (list): A list to store the laser points.
        """
        self.client = RemoteAPIClient()
        self.sim = self.client.getObject('sim')
        self.robot = self.sim.getObject(f'/{robot_name}') 
        self.laser = self.sim.getObject(f'/{robot_name}/fastHokuyo')
        self.robot_name = robot_name
        self.robot_handle = self.sim.getObjectHandle(f'/{robot_name}')
        self.left_wheel = self.sim.getObjectHandle(f'/{robot_name}_leftMotor')
        self.right_wheel = self.sim.getObjectHandle(f'/{robot_name}_rightMotor')
        self.goal_handle = self.sim.getObjectHandle('/ReferenceFrame')
        self.path = []  # Para armazenar o caminho do robô
        self.laser_points = []  # Para armazenar os pontos do laser

    # Função para iniciar a simulação
    def start_simulation(self):
        self.sim.startSimulation()

    # Função para parar a simulação
    def stop_simulation(self):
        self.sim.stopSimulation()

    # Função para retornar a posição do robo
    def get_robot_position(self):
        return self.sim.getObjectPosition(self.robot, -1)

    # Função para retornar a orientação do robo
    def get_robot_orientation(self):
        return self.sim.getObjectOrientation(self.robot, -1)

    # Função para setar a velocidade na roda esquerda do robo
    def set_left_wheel_velocity(self, velocity):
        self.sim.setJointTargetVelocity(self.left_wheel, velocity)

    # Função para setar a velocidade na roda direita do robo
    def set_right_wheel_velocity(self, velocity):
        self.sim.setJointTargetVelocity(self.right_wheel, velocity)
        
    def get_laser_position(self):
        self.sim.getObjectPosition(self.laser, -1)
        
    def get_laser_orientation(self):
        self.sim.getObjectOrientation(self.laser, -1)
    
    def set_goal_position(self, position):
        self.sim.setObjectPosition(self.goal_handle, -1, position)
        
    def update_path(self):
        robot_pos = self.get_robot_position()
        self.path.append(robot_pos)

    def update_laser_points(self, laser_data):
        for point in laser_data:
            self.laser_points.append(point)