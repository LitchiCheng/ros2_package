import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import mujoco
import numpy as np
import glfw

class JointStateSubscriber(Node):
    def __init__(self):
        super().__init__("joint_state_subscriber")
        self.subscription = self.create_subscription(
            JointState,
            "/joint_states",
            self.callback,
            10
        )
        
        # 加载模型
        self.model = mujoco.MjModel.from_xml_path('/home/dar/MuJoCoBin/mujoco_menagerie/franka_emika_panda/scene.xml')
        self.data = mujoco.MjData(self.model)

        # 打印所有 body 的 ID 和名称
        print("All bodies in the model:")
        for i in range(self.model.nbody):
            body_name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, i)
            print(f"ID: {i}, Name: {body_name}")

        # 初始化 GLFW
        if not glfw.init():
            return

        self.window = glfw.create_window(1200, 900, 'Panda Arm Control', None, None)
        if not self.window:
            glfw.terminate()
            return

        glfw.make_context_current(self.window)

        # 设置鼠标滚轮回调函数
        glfw.set_scroll_callback(self.window, self.scroll_callback)

        # 初始化渲染器
        self.cam = mujoco.MjvCamera()
        self.opt = mujoco.MjvOption()
        mujoco.mjv_defaultCamera(self.cam)
        mujoco.mjv_defaultOption(self.opt)
        self.pert = mujoco.MjvPerturb()
        self.con = mujoco.MjrContext(self.model, mujoco.mjtFontScale.mjFONTSCALE_150.value)

        self.scene = mujoco.MjvScene(self.model, maxgeom=10000)

        # 找到末端执行器的 body id
        self.end_effector_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, 'hand')
        print(f"End effector ID: {self.end_effector_id}")
        if self.end_effector_id == -1:
            print("Warning: Could not find the end effector with the given name.")
            glfw.terminate()
            return

        # 初始关节角度
        self.initial_q = self.data.qpos[:7].copy()
        print(f"Initial joint positions: {self.initial_q}")

        self.timer1 = self.create_timer(
            0.01,  # 100ms周期
            self.timer_callback1
        )

    def scroll_callback(self, window, xoffset, yoffset):
        # 调整相机的缩放比例
        self.cam.distance *= 1 - 0.1 * yoffset

    def limit_angle(self, angle):
        while angle > np.pi:
            angle -= 2 * np.pi
        while angle < -np.pi:
            angle += 2 * np.pi
        return angle    

    def timer_callback1(self):
        if not glfw.window_should_close(self.window):
    
            # 获取当前末端执行器位置
            mujoco.mj_forward(self.model, self.data)
            self.end_effector_pos = self.data.body(self.end_effector_id).xpos

            # 设置关节目标位置
            self.data.qpos[:7] = self.positions

            # 模拟一步
            mujoco.mj_step(self.model, self.data)

            # 更新渲染场景
            viewport = mujoco.MjrRect(0, 0, 1200, 900)
            mujoco.mjv_updateScene(self.model, self.data, self.opt, self.pert, self.cam, mujoco.mjtCatBit.mjCAT_ALL.value, self.scene)
            mujoco.mjr_render(viewport, self.scene, self.con)

            # 交换前后缓冲区
            glfw.swap_buffers(self.window)
            glfw.poll_events()

    def callback(self, msg: JointState):
        # 过滤Panda机械臂的7个关节（名称以panda_joint开头）
        panda_joints = [name for name in msg.name if "panda_joint" in name]
        self.positions = [msg.position[i] for i, name in enumerate(msg.name) if "panda_joint" in name]

        for name, pos in zip(panda_joints, self.positions):
            self.get_logger().info(f"{name}: {pos:.4f}")

def main(args=None):
    rclpy.init(args=args)
    node = JointStateSubscriber()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
    # 清理资源
    glfw.terminate()