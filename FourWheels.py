import numpy as np

def DistanceSweptOutByLegsUntilAngle(theta):#If this was a circular wheel, this would be the length of the arc
    Ans = 0.0
    gamma1 = -1.51491088379#angle between edge of flat tip and midline
    gamma2 = -1.02925556679#angle between midline and where circle meets rectangle 
    epsilon1 = 0.0497377466#length between edge of flat tip and axle
    epsilon2 = 0.0253105666#radius of circle part
    epsilon3 = 0.0333769208#length between rectangle's corner and axle
    if(theta < -np.pi/2 or theta > np.pi/2):
        #raise Exception("Angle out of range.")
        return 0
    Ans += epsilon1*(min([gamma1, theta]))
    if(theta < gamma1):
        return Ans
    Ans += epsilon2*(min([gamma2, theta]) - gamma1)
    if(theta < gamma2):
        return Ans
    Ans += epsilon3*(theta - gamma2)
    return Ans

def DistanceSweptOutBetweenAngles(theta1, theta2): #theta2 > theta1
    return abs(DistanceSweptOutByLegsUntilAngle(theta2) - DistanceSweptOutByLegsUntilAngle(theta1))


def calculate_projection(vector, point):#both numpy arrays. Returns a numpy array
    return vector*(np.dot(vector, point)/np.dot(vector, vector))

def calculate_residual(vector, point):
    return vector - calculate_projection(vector, point)


'''
The triangle code below is given in C++ at https://stackoverflow.com/a/2049593/18924580
'''
def sign (p1, p2, p3):
    return (p1[0] - p3[0]) * (p2[1] - p3[1]) - (p2[0] - p3[0]) * (p1[1] - p3[1]);

def PointInTriangle (pt, v1, v2,  v3):

    d1 = sign(pt, v1, v2);
    d2 = sign(pt, v2, v3);
    d3 = sign(pt, v3, v1);

    has_neg = (d1 < 0) or (d2 < 0) or (d3 < 0);
    has_pos = (d1 > 0) or (d2 > 0) or (d3 > 0);

    return not (has_neg and has_pos);

def normalize(v):
    return v/np.linalg.norm(v)

def vectorOrthogonalTo(v):
    return normalize(np.array([-v[1], v[0]]))

class Leg:
    def __init__(self, angle, cg, x, y, direc):#x and y axle coordinates from COM,  which is the origin. direc is direction of rotation: 1 if angle 0 deg is facing front, -1
        #otherwise
        self.angle = min([max([angle, -np.pi/2]), np.pi/2])
        self.cg = cg
        self.x = x
        self.y = y
        self.direction = direc
    def rotateToAngle(self, new_angle):
        self.angle = min([max([new_angle, -np.pi/2]), np.pi/2])
    def getLegSpecs(self):
        return np.array([self.angle, self.cg, self.x, self.y, self.direction])
    def getXY(self):
        return np.array([self.x, self.y])
        
class RCCar:
    def __init__(self, L1, L2, L3, L4, COM): #L1 is a numpy array of arguments for the constructor of Leg(). COM is a 2-elem numpy array of coordinates of COM
        self.Leg1 = Leg(L1[0], L1[1], L1[2], L1[3], L1[4])
        self.Leg2 = Leg(L2[0], L2[1], L2[2], L2[3], L2[4])
        self.Leg3 = Leg(L3[0], L3[1], L3[2], L3[3], L3[4])
        self.Leg4 = Leg(L4[0], L4[1], L4[2], L4[3], L4[4])
        self.COM = COM
        self.distance_traveled = 0
        self.initial_angle1 = L1[0]
        self.initial_angle2 = L2[0]
        self.initial_angle3 = L3[0]
        self.initial_angle4 = L4[0]
    def rotateLegs(self, x_q):#x_q determines rotation of wheels
        '''
        print("Leg1:")
        print(self.Leg1.getLegSpecs())
        print("Leg2:")
        print(self.Leg2.getLegSpecs())
        print("Leg3:")
        print(self.Leg3.getLegSpecs())
        print("Leg4:")
        print(self.Leg4.getLegSpecs())
        '''
        
        if(x_q >= 0):#counterclockwise
            max_angle_to_rot1 = abs(self.Leg1.angle - (-np.pi/2*self.Leg1.direction))
            max_angle_to_rot2 = abs(self.Leg2.angle - (-np.pi/2*self.Leg2.direction))
            max_angle_to_rot3 = abs(self.Leg3.angle - (-np.pi/2*self.Leg3.direction))
            max_angle_to_rot4 = abs(self.Leg4.angle - (-np.pi/2*self.Leg4.direction))
            angle_to_rot = min([max_angle_to_rot1, max_angle_to_rot2, max_angle_to_rot3, max_angle_to_rot4])
            dist_rot_causes1 = DistanceSweptOutBetweenAngles(self.Leg1.angle, self.Leg1.angle - self.Leg1.direction*angle_to_rot)
            dist_rot_causes2 = DistanceSweptOutBetweenAngles(self.Leg2.angle, self.Leg2.angle - self.Leg2.direction*angle_to_rot)
            dist_rot_causes3 = DistanceSweptOutBetweenAngles(self.Leg3.angle, self.Leg3.angle - self.Leg3.direction*angle_to_rot)
            dist_rot_causes4 = DistanceSweptOutBetweenAngles(self.Leg4.angle, self.Leg4.angle - self.Leg4.direction*angle_to_rot)
            motion_vector1 = vectorOrthogonalTo(self.Leg1.getXY())
            motion_vector2 = vectorOrthogonalTo(self.Leg2.getXY())
            motion_vector3 = vectorOrthogonalTo(self.Leg3.getXY())
            motion_vector4 = vectorOrthogonalTo(self.Leg4.getXY())
            resultant_vector = motion_vector1*dist_rot_causes1 + motion_vector2*dist_rot_causes2 + motion_vector3*dist_rot_causes3 + motion_vector4*dist_rot_causes4
            #self.distance_traveled = self.distance_traveled + max([dist_rot_causes1, dist_rot_causes2, dist_rot_causes3, dist_rot_causes4])
            self.distance_traveled = self.distance_traveled + abs(np.dot(resultant_vector, np.array([0,1])))
            self.Leg1.rotateToAngle(self.Leg1.angle - self.Leg1.direction*angle_to_rot_front)
            self.Leg2.rotateToAngle(self.Leg2.angle - self.Leg2.direction*angle_to_rot_front)
            self.Leg3.rotateToAngle(self.Leg3.angle - self.Leg3.direction*angle_to_rot_back)
            self.Leg4.rotateToAngle(self.Leg4.angle - self.Leg4.direction*angle_to_rot_back)
            
        if(x_q < 0):#clockwise
            max_angle_to_rot1 = abs(self.Leg1.angle - (np.pi/2*self.Leg1.direction))
            max_angle_to_rot2 = abs(self.Leg2.angle - (np.pi/2*self.Leg2.direction))
            max_angle_to_rot3 = abs(self.Leg3.angle - (np.pi/2*self.Leg3.direction))
            max_angle_to_rot4 = abs(self.Leg4.angle - (np.pi/2*self.Leg4.direction))
            angle_to_rot = min([max_angle_to_rot1, max_angle_to_rot2, max_angle_to_rot3, max_angle_to_rot4])
            dist_rot_causes1 = DistanceSweptOutBetweenAngles(self.Leg1.angle, self.Leg1.angle + self.Leg1.direction*angle_to_rot)
            dist_rot_causes2 = DistanceSweptOutBetweenAngles(self.Leg2.angle, self.Leg2.angle + self.Leg2.direction*angle_to_rot)
            dist_rot_causes3 = DistanceSweptOutBetweenAngles(self.Leg3.angle, self.Leg3.angle + self.Leg3.direction*angle_to_rot)
            dist_rot_causes4 = DistanceSweptOutBetweenAngles(self.Leg4.angle, self.Leg4.angle + self.Leg4.direction*angle_to_rot)
            motion_vector1 = vectorOrthogonalTo(self.Leg1.getXY())
            motion_vector2 = vectorOrthogonalTo(self.Leg2.getXY())
            motion_vector3 = vectorOrthogonalTo(self.Leg3.getXY())
            motion_vector4 = vectorOrthogonalTo(self.Leg4.getXY())
            resultant_vector = motion_vector1*dist_rot_causes1 + motion_vector2*dist_rot_causes2 + motion_vector3*dist_rot_causes3 + motion_vector4*dist_rot_causes4
            #self.distance_traveled = self.distance_traveled + max([dist_rot_causes1, dist_rot_causes2, dist_rot_causes3, dist_rot_causes4])
            self.distance_traveled = self.distance_traveled - abs(np.dot(resultant_vector, np.array([0,1])))
            self.Leg1.rotateToAngle(self.Leg1.angle + self.Leg1.direction*angle_to_rot_front)
            self.Leg2.rotateToAngle(self.Leg2.angle + self.Leg2.direction*angle_to_rot_front)
            self.Leg3.rotateToAngle(self.Leg3.angle + self.Leg3.direction*angle_to_rot_back)
            self.Leg4.rotateToAngle(self.Leg4.angle + self.Leg4.direction*angle_to_rot_back)
            #print(angle_to_rot)
    def updateContactsGround(self):

        if(self.Leg1.angle > -np.pi/4):
            self.Leg1.cg = 0
        else:
            self.Leg1.cg = 1
        if(self.Leg2.angle > -np.pi/4):
            self.Leg2.cg = 0
        else:
            self.Leg2.cg = 1
        if(self.Leg3.angle > -np.pi/4):
            self.Leg3.cg = 0
        else:
            self.Leg3.cg = 1
        if(self.Leg4.angle > -np.pi/4):
            self.Leg4.cg = 0
        else:
            self.Leg4.cg = 1
            
        contacts_ground_array = np.array([self.Leg1.cg, self.Leg2.cg, self.Leg3.cg, self.Leg4.cg])
        axle_coordinates_array = np.array([[self.Leg1.x, self.Leg1.y], [self.Leg2.x, self.Leg2.y], [self.Leg3.x, self.Leg3.y], [self.Leg4.x, self.Leg4.y]])
        num_contacts_ground = np.sum(contacts_ground_array)
        if(num_contacts_ground == 1 or num_contacts_ground == 0):
            print("ONE OR ZERO")
            '''
            self.Leg1.cg = 0
            self.Leg2.cg = 0
            self.Leg3.cg = 0
            self.Leg4.cg = 0
            
            '''
            
            rcg = np.random.choice([0,1], size = 4)
            self.Leg1.cg = rcg[0]
            self.Leg2.cg = rcg[1]
            self.Leg3.cg = rcg[2]
            self.Leg4.cg = rcg[3]
            
            '''
            self.Leg1.cg = 0.5
            self.Leg2.cg = 0.5
            self.Leg3.cg = 0.5
            self.Leg4.cg = 0.5
            '''
            
            self.Leg1.rotateToAngle(0)
            self.Leg2.rotateToAngle(0)
            self.Leg3.rotateToAngle(0)
            self.Leg4.rotateToAngle(0)
        if(num_contacts_ground == 2):
            print("TWO")
            points_touching_ground = axle_coordinates_array[np.nonzero(contacts_ground_array == 1)]
            fulcrum_line = points_touching_ground[0] - points_touching_ground[1]
            translated_COM = self.COM - points_touching_ground[1]
            translated_COM_to_fulcrum_line = calculate_residual(fulcrum_line, translated_COM)
            indexes_not_touching_ground = np.nonzero(contacts_ground_array == 0)[0]
            #print(indexes_not_touching_ground)
            not_touching1_to_fulcrum_line = calculate_residual(fulcrum_line, axle_coordinates_array[indexes_not_touching_ground[0]])
            not_touching2_to_fulcrum_line = calculate_residual(fulcrum_line, axle_coordinates_array[indexes_not_touching_ground[1]])
            touch_after1 = False
            touch_after2 = False
            if(np.dot(not_touching1_to_fulcrum_line, translated_COM_to_fulcrum_line) > 0):
                touch_after1 = True
            if(np.dot(not_touching2_to_fulcrum_line, translated_COM_to_fulcrum_line) > 0):
                touch_after2 = True
            if(touch_after1 and touch_after2):
                if(indexes_not_touching_ground[0] == 0):
                    self.Leg1.cg = 1
                elif(indexes_not_touching_ground[0] == 1):
                    self.Leg2.cg = 1
                elif(indexes_not_touching_ground[0] == 2):
                    self.Leg3.cg = 1
                elif(indexes_not_touching_ground[0] == 3):
                    self.Leg4.cg = 1
                else:
                    pass
                if(indexes_not_touching_ground[1] == 0):
                    self.Leg1.cg = 1
                elif(indexes_not_touching_ground[1] == 1):
                    self.Leg2.cg = 1
                elif(indexes_not_touching_ground[1] == 2):
                    self.Leg3.cg = 1
                elif(indexes_not_touching_ground[1] == 3):
                    self.Leg4.cg = 1
                else:
                    pass
            elif(touch_after1 and not touch_after2):
                if(indexes_not_touching_ground[0] == 0):
                    self.Leg1.cg = 1
                elif(indexes_not_touching_ground[0] == 1):
                    self.Leg2.cg = 1
                elif(indexes_not_touching_ground[0] == 2):
                    self.Leg3.cg = 1
                elif(indexes_not_touching_ground[0] == 3):
                    self.Leg4.cg = 1
                else:
                    pass
                if(indexes_not_touching_ground[1] == 0):
                    self.Leg1.cg = 0
                elif(indexes_not_touching_ground[1] == 1):
                    self.Leg2.cg = 0
                elif(indexes_not_touching_ground[1] == 2):
                    self.Leg3.cg = 0
                elif(indexes_not_touching_ground[1] == 3):
                    self.Leg4.cg = 0
                else:
                    pass
            elif(not touch_after1 and touch_after2):
                if(indexes_not_touching_ground[0] == 0):
                    self.Leg1.cg = 0
                elif(indexes_not_touching_ground[0] == 1):
                    self.Leg2.cg = 0
                elif(indexes_not_touching_ground[0] == 2):
                    self.Leg3.cg = 0
                elif(indexes_not_touching_ground[0] == 3):
                    self.Leg4.cg = 0
                else:
                    pass
                if(indexes_not_touching_ground[1] == 0):
                    self.Leg1.cg = 1
                elif(indexes_not_touching_ground[1] == 1):
                    self.Leg2.cg = 1
                elif(indexes_not_touching_ground[1] == 2):
                    self.Leg3.cg = 1
                elif(indexes_not_touching_ground[1] == 3):
                    self.Leg4.cg = 1
                else:
                    pass
            else:
                raise Exception("COM is outside RC Car????")
        if(num_contacts_ground == 3):
            print("THREE")
            points_touching_ground = axle_coordinates_array[np.nonzero(contacts_ground_array == 1)]
            index_of_point_not_touching_ground = np.nonzero(contacts_ground_array == 0)[0]
            point_not_touching_ground = axle_coordinates_array[index_of_point_not_touching_ground]
            distances_from_not_touching_to_touching = np.array([np.linalg.norm(points_touching_ground[0]- point_not_touching_ground), np.linalg.norm(points_touching_ground[1]- point_not_touching_ground), np.linalg.norm(points_touching_ground[2]- point_not_touching_ground)])
            index_of_point_farthest_from_that_not_touching = np.argmin(distances_from_not_touching_to_touching)
            #print(index_of_point_farthest_from_that_not_touching)
            isInTri = PointInTriangle(self.COM, points_touching_ground[0], points_touching_ground[1], points_touching_ground[2])
            if(not isInTri):
                if(index_of_point_not_touching_ground == 0):
                    self.Leg1.cg = 1
                elif(index_of_point_not_touching_ground == 1):
                    self.Leg2.cg = 1
                elif(index_of_point_not_touching_ground == 2):
                    self.Leg3.cg = 1
                elif(index_of_point_not_touching_ground == 3):
                    self.Leg4.cg = 1
                else:
                    pass
                if(index_of_point_farthest_from_that_not_touching == 0):
                    self.Leg1.cg = 0
                elif(index_of_point_farthest_from_that_not_touching == 1):
                    self.Leg2.cg = 0
                elif(index_of_point_farthest_from_that_not_touching == 2):
                    self.Leg3.cg = 0
                elif(index_of_point_farthest_from_that_not_touching == 3):
                    self.Leg4.cg = 0
                else:
                    pass
            else:
                pass
        if(num_contacts_ground == 4):
            print("FOUR")
            pass

#  L1      L2
#  L3      L4

#    front
#left     right
#    back


x1 = 7.5
y1 = 8
x2 = 7.5
y2 = -8
x3 = -6
y3 = -8
x4 = -6
y4 = 8.5



packets_memory = []#Each packet is the observation at time t, the action, the reward, and the order of the action in the sequence
LOLLOL = 2
MSL = 9#max seq length
def fill_packets():
    max_seq_length = 9#Because of telephone numbers. The user should be able to remember the sequence.
    MSL = max_seq_length
    for i in range(100):
        rsa = np.random.uniform(-np.pi/2, 0, 4)#Random Starting Angles
        rcg = np.random.choice([0,1], size = 4)
        the_RC_car = RCCar(np.array([rsa[0], rcg[0], x1,y1,1]),np.array([rsa[1], rcg[1],x2,y2,1]),np.array([rsa[2], rcg[2],x3,y3,-1]),np.array([rsa[3], rcg[3],x4,y4,-1]), np.array([0,0]))
        the_RC_car.updateContactsGround()
        print("New Simulation")
        random_seq = np.sign(np.random.uniform(low = -1, high = 1, size = max_seq_length))
        #print(random_seq)
        for j in range(len(random_seq)):
            packet = []
            LOL = len(the_RC_car.Leg1.getLegSpecs())
            LOLLOL = LOL
            '''
            for i in range(LOL):
                packet.append(the_RC_car.Leg1.getLegSpecs()[i])
            for i in range(LOL):
                packet.append(the_RC_car.Leg2.getLegSpecs()[i])
            for i in range(LOL):
                packet.append(the_RC_car.Leg3.getLegSpecs()[i])
            for i in range(LOL):
                packet.append(the_RC_car.Leg4.getLegSpecs()[i])
            '''
            packet.append(the_RC_car.initial_angle1)
            packet.append(the_RC_car.initial_angle2)
            packet.append(the_RC_car.initial_angle3)
            packet.append(the_RC_car.initial_angle4)
            
            packet.append(the_RC_car.Leg1.getLegSpecs()[1])
            packet.append(the_RC_car.Leg2.getLegSpecs()[1])
            packet.append(the_RC_car.Leg3.getLegSpecs()[1])
            packet.append(the_RC_car.Leg4.getLegSpecs()[1])
            
            packet.append(random_seq[j])
            
            prior_distance_traveled = the_RC_car.distance_traveled
            the_RC_car.rotateLegs(random_seq[j])
            the_RC_car.updateContactsGround()
            reward = the_RC_car.distance_traveled - prior_distance_traveled
            
            packet.append(reward)
            packet.append(j)
            packets_memory.append(packet)

fill_packets()

import torch
from torch import nn


device = (
    "cuda"
    if torch.cuda.is_available()
    else "mps"
    if torch.backends.mps.is_available()
    else "cpu"
)


class NeuralNetwork(nn.Module):
    def __init__(self):
        super().__init__()
        self.flatten = nn.Flatten()
        self.full_neural_network = nn.Sequential(
            nn.Linear(len(packets_memory[0]) - 1, 2),#not including reward as input
            nn.ReLU(),
            nn.Linear(2, 1),#2 -- one for state, one for action
            nn.Sigmoid(),
        )
    def forward(self, x):
        x = self.flatten(x)
        max_possible_dist_traveled = DistanceSweptOutBetweenAngles(-np.pi/2, np.pi/2)
        expectedReward = max_possible_dist_traveled*self.full_neural_network(x)
        return expectedReward

model = NeuralNetwork().to(device)

#print(packets_memory)
data = np.array(packets_memory)
#print(data.shape)


#reward_index = 4*LOLLOL + 1 + 1 - 1#check if you update Leg constructor
reward_index = 9
y = torch.tensor(data[:,reward_index].tolist())
xx = torch.tensor(np.delete(data, reward_index, 1).tolist())

print("All good so far. Now for training.")
print(data)


loss_fn = torch.nn.MSELoss(reduction='sum')
learning_rate = 0.01
optimizer = torch.optim.Adam(model.parameters(), lr=learning_rate)
for t in range(2000):
    y_pred = model(xx)
    loss = loss_fn(y_pred, y)
    #if t % 100 == 99:
    if(True):
        print(t, loss.item())
    optimizer.zero_grad()
    loss.backward()
    optimizer.step()

final_y_pred = model(xx)
argmax_of_final_y_pred = torch.argmax(final_y_pred).item()
best_packet = packets_memory[argmax_of_final_y_pred]
best_starting_angles = best_packet[0:4]
best_sequence = []
mod_MSL = argmax_of_final_y_pred%MSL
for i in range(argmax_of_final_y_pred - mod_MSL, argmax_of_final_y_pred - mod_MSL + 9):
    best_sequence.append(packets_memory[i][8])

print(57.3248407643*np.array(best_starting_angles))
print(best_sequence)
