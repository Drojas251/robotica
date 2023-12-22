import fcl
import numpy as np

class CollisionObjs:
    def __init__(self):
        self.collision_objs = []
        self.collsion_manager = None

        self._geom_obj = []
        self._obj_names = []

        self.geom_id_to_obj = {}
        self.geom_id_to_name = {}


    def create_box_collision_obj(self, name, l, h, R, T):
        # This function makes a box primitve collision object
        box = fcl.Box(l,h,0.1)
        transform = fcl.Transform(R, T)
        collision_obj = fcl.CollisionObject(box, transform)

        self._geom_obj.append(box)
        self._obj_names.append(name)
        self.collision_objs.append(collision_obj)

    def make_collision_manager(self):
        # Create mappings for obs
        self._map_obj_to_obj_names()

        manager = fcl.DynamicAABBTreeCollisionManager()
        manager.registerObjects(self.collision_objs)
        manager.setup()
        self.collsion_manager = manager

    def get_geom_name(self, coll_geom):
        return self.geom_id_to_name[id(coll_geom)]

    def _map_obj_to_obj_names(self):
        # Create map from geometry IDs to objects
        self.geom_id_to_obj = { id(geom) : obj for geom, obj in zip(self._geom_obj, self.collision_objs) }

        # Create map from geometry IDs to string names
        self.geom_id_to_name = { id(geom) : name for geom, name in zip(self._geom_obj, self._obj_names) }

class EnvCollisionObjs(CollisionObjs):
    def __init__(self):
        CollisionObjs.__init__(self)

    def add_box_collision_obj(self, name, l, h, T):
        R = [[1,0,0], [0,1,0], [0,0,1]]
        self.create_box_collision_obj(name, l, h, R, T)

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
            self.create_box_collision_obj(f"link{i}", self.link_lengths[i], 0.005, R, T)
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

        self.collision_callback_func = None

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
                env_obj = self.env_collision_objs.get_geom_name(c.o1)
                robot_link = self.robot_collision_objs.get_geom_name(c.o2)
                self._invoke_collision_callback(env_obj, robot_link)
            
            return True

        return False

    def add_collision_callback(self, func):
        self.collision_callback_func = func

    def _invoke_collision_callback(self, *args):
        self.collision_callback_func(*args)


