import rospy
import numpy as np
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from queue import PriorityQueue

class AStar:
    def __init__(self, grid):
        rospy.init_node('a_star_path_planner', anonymous=True)
        self.path_pub = rospy.Publisher('/a_star_path', Path, queue_size=10)
        self.grid = grid  # 장애물 맵
        self.rate = rospy.Rate(1)  # 1 Hz

    def is_valid(self, x, y):
        # 그리드 내에서 유효한 위치인지 확인하고 장애물이 있는지 체크
        if 0 <= x < self.grid.shape[0] and 0 <= y < self.grid.shape[1]:
            return self.grid[x][y] == 0  # 0이 비어 있는 공간, 1이 장애물
        return False

    def heuristic(self, a, b):
        # 맨해튼 거리 계산
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def plan_path(self, start, goal):
        # A* 알고리즘 시작 메시지 출력
        rospy.loginfo("A* 알고리즘 시작: 시작점: %s, 목표점: %s", start, goal)
        
        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = rospy.Time.now()

        # A* 알고리즘
        open_set = PriorityQueue()
        open_set.put((0, start))
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, goal)}

        while not open_set.empty():
            current = open_set.get()[1]

            if current == goal:
                # 경로 생성
                self.reconstruct_path(came_from, current, path)
                self.path_pub.publish(path)
                return

            neighbors = [(current[0]+1, current[1]), (current[0]-1, current[1]),
                         (current[0], current[1]+1), (current[0], current[1]-1)]

            for neighbor in neighbors:
                if self.is_valid(neighbor[0], neighbor[1]):
                    tentative_g_score = g_score[current] + 1
                    if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                        came_from[neighbor] = current
                        g_score[neighbor] = tentative_g_score
                        f_score[neighbor] = tentative_g_score + self.heuristic(neighbor, goal)
                        if neighbor not in [i[1] for i in open_set.queue]:
                            open_set.put((f_score[neighbor], neighbor))

        rospy.loginfo("경로를 찾을 수 없음")
        return

    def reconstruct_path(self, came_from, current, path):
        while current in came_from:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = rospy.Time.now()
            pose.pose.position.x = current[0]
            pose.pose.position.y = current[1]
            pose.pose.position.z = 0
            path.poses.append(pose)
            current = came_from[current]
        path.poses.reverse()  # 경로를 뒤집음

if __name__ == '__main__':
    try:
        # 그리드 맵 예시 (0: 비어있는 공간, 1: 장애물)
        grid_map = np.array([[0, 0, 0, 0, 0],
                             [0, 1, 1, 0, 0],
                             [0, 0, 0, 0, 0],
                             [0, 0, 1, 1, 0],
                             [0, 0, 0, 0, 0]])
        a_star = AStar(grid_map)

        # 시작점과 목표점 설정 (x, y)
        start_point = (0, 0)
        goal_point = (4, 4)

        a_star.plan_path(start_point, goal_point)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
