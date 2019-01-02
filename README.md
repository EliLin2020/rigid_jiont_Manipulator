# rigid_jiont_Manipulator 二自由度机械臂轨道追踪问题
## 动力学模型 
### 假设 
1. 关节点(joint)刚性连接，不存在弹性振动，非线性振动等问题. 
2. 考虑到该机械臂工作于太空中，忽略重力的影响. 
3. 两连杆(link)密度均匀，且线密度相等.
### 公式 
- $ M(q) \ddot{q}+C(q,\dot{q}) \dot{q}= \tau $ 
- 其中，$ M(q)= $ 
$$ 
\bigl[\begin{matrix}
m_{1}l_{c1}^{2}+m_{2}(l_{1}^{2}+l_{c2}^{2}+2l_{1}l_{c2} \cos q_{2})+I_{1}+I_{2} & m_{2}(l_{c2}^{2}+l_{1}l_{c2} \cos q_{2})+I_{2} 
\\ ;
m_{2}(l_{c2}^{2}+l_{1}l_{c2} \cos q_{2})+I_{2} & m_{2}l_{c2}^{2}+I_{2} 
\end{matrix}\bigr] 
$$ 
- $ C(q,\dot{q})= -m_{2}l_{1}l_{c2}\sin q_{2} $ 
$$
\bigl[\begin{matrix}
\dot(q)_{2} & \dot(q)_{1}+\dot(q)_{2} \\ ; -\dot(q)_{1} & 0
\end{matrix}\bigr] 
$$
## PID控制率 
## 程序思路
