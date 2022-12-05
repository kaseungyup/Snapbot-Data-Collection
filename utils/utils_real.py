import numpy as np
from realworld_func.class_motionhelper import timer

def get_traj(qpos):
    qpos.T[1] = -qpos.T[1]
    qpos.T[3] = -qpos.T[3]
    qpos.T[5] = -qpos.T[5]
    qpos.T[7] = -qpos.T[7]
    qpos = 2048/180*qpos+2048

    # 41 degree 
    plus_joint_limit = 2048/180*41 + 2048
    minus_joint_limit = -2048/180*41 + 2048
    qpos[:, 0] = np.clip(qpos[:, 0], minus_joint_limit, plus_joint_limit)
    qpos[:, 2] = np.clip(qpos[:, 2], minus_joint_limit, plus_joint_limit)
    qpos[:, 4] = np.clip(qpos[:, 4], minus_joint_limit, plus_joint_limit)
    qpos[:, 6] = np.clip(qpos[:, 6], minus_joint_limit, plus_joint_limit)
    return qpos


def run_snapbot(qpos, snapbot, Hz, max_sec):
    traj = get_traj(qpos)
    t = timer(_HZ=Hz, _MAX_SEC=max_sec)
    t.start()
    idx, threshold= 0, 0
    flag = False
    while t.is_notfinished():
        if t.do_run():
            pos = traj[idx]
            if not flag:
                flag = True
                threshold = pos
            idx = idx + 1
            snapbot.set_goalpos((np.array(pos, dtype=np.int32).tolist()))
            if idx == traj.shape[0]:
                t.finish()
    print("FINISHED")

def run_snapbot_single(qpos, snapbot):
    qpos.T[1] = -qpos.T[1]
    qpos.T[3] = -qpos.T[3]
    qpos.T[5] = -qpos.T[5]
    qpos.T[7] = -qpos.T[7]
    qpos = 2048/180*qpos+2048

    plus_joint_limit = 2048/180*41 + 2048
    minus_joint_limit = -2048/180*41 + 2048
    qpos[0] = np.clip(qpos[0], minus_joint_limit, plus_joint_limit)
    qpos[2] = np.clip(qpos[2], minus_joint_limit, plus_joint_limit)
    qpos[4] = np.clip(qpos[4], minus_joint_limit, plus_joint_limit)
    qpos[6] = np.clip(qpos[6], minus_joint_limit, plus_joint_limit)

    traj = qpos
    threshold = 0
    flag = False
    pos = traj
    if not flag:
        flag = True
        threshold = pos
    snapbot.set_goalpos((np.array(pos, dtype=np.int32)))


