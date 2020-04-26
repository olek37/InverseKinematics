import numpy as np

class Model(object):
  lengths = []
  #pelvis, femur, tibia, heel (foot) 

  desired_height = 7
  #desired pelvis height

  current_height = 0
  #current pelvis height

  side = 0
  #assume left is the starting side

  rotations = []
  #rotation of each joint along axis parallel to the ground
  #assume pelvis is always parallel to ground
  #joints: pelvis-femur, femur-tibia, tibia-heel (pf, ft, th)

  pf_rotation = 0
  #rotation of pf joint along axis perpendicular to the ground 

  pelvis_rotation = 0
  #pelvis rotation from the starting position

  th_position = []
  #foot position on the ground

  pf_position = []
  #pelvis-femur joint position projected onto the ground

  def __init__(self, lengths, desired_height):
    self.lengths = lengths
    self.desired_height = desired_height
    self.th_position = [
      [0,0],
      #left
      [lengths[0], 0]
      #right
      ]
    self.pf_position = [
      [0,0],
      #left
      [lengths[0], 0]
      #right
      ]
    self.current_height = sum(lengths[1:2])
    self.pf_rotation = 90 
    self.rotations = [
      [
        90, 180, 90
      ],
      [
        90, 180, 90
      ]
    ]
    self.pelvis_rotation = 0
    #set initial stance

  def make_step(self, end_pos):
    self.set_optimal_pelvis_rotation(end_pos)
    self.set_pf_rotation(end_pos)
    old_rot = self.rotations[:]
    self.set_rotations(end_pos)
    new_rot = self.rotations[:]
    animate(old_rot, new_rot, 50)
    
  def set_rotations(self, end_pos):
    stat_pos = self.th_position[1 - self.side]
    pf_th = get_dist(end_pos, get_mid(end_pos, stat_pos))
    pf_th_3d = np.sqrt(pf_th**2 + self.current_height**2)

    alpha = np.rad2deg(np.arccos(sss_solve(self.lengths[1], self.lengths[2], pf_th_3d)))
    #calculate the x-axis angle of femur-tibia joint

    beta = np.rad2deg(np.arccos(sss_solve(self.lengths[1], pf_th_3d, self.lengths[2])))
    #pelvis-femur x-axis angle

    gamma = 180 - (alpha + beta)
    #tibia-foot x-axis angle

    angles = [[alpha, beta, gamma], [-1 * alpha, beta, 180 - gamma]]

    if self.side == 0:
      self.rotations = angles
    else:
      self.rotations = angles[::-1]    

  def set_pf_rotation(self, end_pos):
    #objective: find the pf_rotation
    stat_pos = self.th_position[1 - self.side]
    
    pc_th = get_dist(end_pos, get_mid(end_pos, stat_pos))
    #distance from pelvis center to end heel

    pf_th = get_dist(end_pos, self.pf_position[self.side])
    #distance from active leg pf joint to end heel

    pc_pf = self.lengths[0]/2
    #distance from pelvis center to active leg pf joint

    self.pf_rotation = np.rad2deg(np.arccos(sss_solve(pf_th, pc_pf, pc_th)))

  def set_optimal_pelvis_rotation(self, end_pos):
    #objective: find the pelvis
    #constraints: 1. minimal pelvis rotation, 2. minimal difference between current height and desired height

    stat_pos = self.th_position[1 - self.side]
    end_pelvis_center = get_mid(end_pos, stat_pos)
    pelvis_len = self.lengths[0]

    for angle in range(self.pelvis_rotation, self.pelvis_rotation + 90):
      #calculate the pf positon of the active leg
      for angle_coeff in [-1, 1]:
        alpha = angle * angle_coeff
        x = np.sin(alpha) * pelvis_len/2
        y = np.cos(alpha) * pelvis_len/2
        pf_active_pos = [
          end_pelvis_center[0] + x,
          end_pelvis_center[1] + y
        ]
        pf_stat_pos = [
          end_pelvis_center[0] - x,
          end_pelvis_center[1] - y
        ]
        if self.set_optimal_height(pf_active_pos, pf_stat_pos, alpha):
          return

  def set_optimal_height(self, pf_active_pos, pf_stat_pos, alpha):
    #let's find a height near the desired height that allows for this step at angle alpha
    for i in np.arange(0, self.desired_height, 0.1):
      for dist_coeff in [-1, 1]:
        height_change = i * dist_coeff
        temp_dist = np.sqrt(get_dist(pf_active_pos, self.th_position[self.side])**2 + (self.desired_height + height_change)**2)
        if temp_dist < (self.lengths[1] + self.lengths[2]):
          self.current_height = self.desired_height + height_change
          self.pf_rotation = alpha
          position = [
            pf_active_pos,
            pf_stat_pos
          ]
          if self.side:
            self.pf_position = position
          else:
            self.pf_position = position[::-1]
          return 1
      
def sss_solve(a, b, c):
  return (a**2 + b**2 - c**2)/(2*a*b)

def get_nearest(p1, p2, r):
  x = p2[0] - p1[0]
  y = p2[1] - p1[1]
  mag = np.sqrt(x**2 + y**2)
  return [p2[0] + x/mag * r, p2[1] + y/mag * r]

def get_mid(p1, p2):
    return [(p1[0]+p2[0])/2, (p1[1]+p2[1])/2]

def get_dist(p1, p2):
    return np.sqrt((p2[0]-p1[0])**2 + (p2[1]-p1[1])**2)

def animate(start_rot, end_rot, time):
  differences = []
  for (side_start, side_end) in zip(start_rot, end_rot):
    side = []
    for (angle_start, angle_end) in zip(side_start, side_end):
      side.append((angle_end - angle_start)/time)
    differences.append(side)
  #find the difference between angles that will be added each time interval
  frame = np.array(start_rot)
  frames = [frame]
  for i in range(1, time):
    frame = np.add(frame, np.array(differences))
    frames.append(frame)
  print(frames)

model = Model([1, 4, 3, 1], 7)

model.make_step([5,6])
model.make_step([7,6])
model.make_step([8,10])
