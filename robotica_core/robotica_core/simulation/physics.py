import fcl
import numpy as np

class CollisionObjs:
    def __init__(self):
        self.collision_objs = []
        self.collsion_manager = None

    def create_box_collision_obj(self,l, h, R, T):
        # This function makes a box primitve collision object
        box = fcl.Box(l,h,0.1)
        transform = fcl.Transform(R, T)
        collision_obj = fcl.CollisionObject(box, transform)
        return collision_obj

    def add_collision_obj(self, collision_obj):
        self.collision_objs.append(collision_obj)

    def make_collision_manager(self):
        # This function makes a collision manager for a group of collision objects 
        manager = fcl.DynamicAABBTreeCollisionManager()
        manager.registerObjects(self.collision_objs)
        manager.setup()
        self.collsion_manager = manager

class EnvCollisionObjs(CollisionObjs):
    def __init__(self):
        CollisionObjs.__init__(self)

    def add_box_collision_obj(self, l, h, T):
        R = [[1,0,0], [0,1,0], [0,0,1]]
        coll_obj = self.create_box_collision_obj(l, h, R, T)
        self.add_collision_obj(coll_obj)

class RobotCollisionObjs(CollisionObjs):
    def __init__(self, init_link_transforms, link_lengths):
        CollisionObjs.__init__(self)
        self.link_transforms = init_link_transforms
        self.link_lengths = link_lengths
        self.num_links = len(link_lengths)
        self._init_collision_obj()

    def _init_collision_obj(self):
        for i in range(self.num_links):
            tf = self.link_transforms[i]
            R = tf[:3, :3]
            T = tf[:3, -1]
            link_coll_obj = self.create_box_collision_obj(self.link_lengths[i], 0.005, R, T)
            self.add_collision_obj(link_coll_obj)
        self.make_collision_manager()

    def update_transforms(self, tf_list):
        for i in range(self.num_links):
            R = tf_list[i][:3, :3]
            T = tf_list[i][:3, -1]
            tf = fcl.Transform(R, T)
            self.collision_objs[i].setTransform(tf)

class CollisionManager:
    def __init__(self, init_link_transforms, link_lengths):
        self.robot_collision_objs = RobotCollisionObjs(init_link_transforms, link_lengths)
        self.env_collision_objs = EnvCollisionObjs()
        self.collision_data = None

    def update_robot_tfs(self, tf_list):
        self.robot_collision_objs.update_transforms(tf_list)

    def init_collision_managers(self):
        self.env_collision_objs.make_collision_manager()
        self.robot_collision_objs.make_collision_manager()

    def check_collision(self):
        req = None
        collision_data = None

        self.robot_collision_objs.make_collision_manager()

        req = fcl.CollisionRequest(enable_contact=True)
        collision_data = fcl.CollisionData(request = req)
        self.env_collision_objs.collsion_manager.collide(self.robot_collision_objs.collsion_manager, collision_data, fcl.defaultCollisionCallback)

        in_collision = collision_data.result.is_collision
        if in_collision:
            print(f"Collision Detected!")
            for c in collision_data.result.contacts:
                print(f"{c.o1} and {c.o2}")

