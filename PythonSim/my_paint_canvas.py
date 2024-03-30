# import yaml
import math

from map import Map
from simulator import Simulator
from lib.settings import lane_width, turn_radius, arm_len, NS_lane_count, EW_lane_count, veh_dt, disp_dt, simu_t, time_wrap

from PyQt5.QtCore import Qt, QTimer, QPointF, QRectF, QLineF
from PyQt5.QtGui import QPainter, QColor, QPen, QFont
from PyQt5.QtWidgets import QWidget

class MyPaintCanvas(QWidget):
    def __init__(self, parent=None, mainw=None):
        super().__init__(parent)
        self.mainw = mainw

        self.disp_timer = QTimer(self)
        self.disp_timer.start(int(disp_dt * 1000))
        self.disp_timer.timeout.connect(self.update) # Each update triggers paintEvent
        self.veh_timer = QTimer(self)
        self.veh_timer.start(int(veh_dt * 1000 / time_wrap))
        self.veh_timer.timeout.connect(self.update_traffic)

        self.lw = lane_width
        self.tr = turn_radius
        self.al = arm_len
        self.NSl = NS_lane_count
        self.EWl = EW_lane_count

        self.draw_road_shape = self.gen_draw_road()
        self.draw_traj_shape = self.gen_draw_traj(Map.getInstance().ju_track_table)

       # Set background color
        self.setAutoFillBackground(True)
        palette = self.palette()
        palette.setColor(self.backgroundRole(), QColor(247, 232, 232))
        self.setPalette(palette)

    def update_traffic(self):
        Simulator.getInstance().update()
        if Simulator.getInstance().get_sim_over():
            print('Simulation finished',Simulator.getInstance().get_sim_over())
            self.veh_timer.stop()
            self.disp_timer.stop()
            self.mainw.close()



    def paintEvent(self, event): # Called every time disp_timer timeout, redraw
        qp = QPainter(self)
        qp.setRenderHint(QPainter.Antialiasing, True)  # Anti-aliasing

        window_wid = (self.lw * self.NSl + self.tr + self.al) * 2
        window_hgt = (self.lw * self.EWl + self.tr + self.al) * 2
        if window_hgt / self.height() < window_wid / self.width(): 
            # The height direction is relatively loose and the width is full.
            viewport_wid = self.width()
            viewport_hgt = self.width() * window_hgt / window_wid
        else:
            # The width direction is relatively loose and the height is full.
            viewport_wid = self.height() * window_wid / window_hgt 
            viewport_hgt = self.height()
        # setViewport(x, y, w, h) sets the coordinates of the window on the control, here it is centered
        qp.setViewport(int((self.width() - viewport_wid) / 2), int((self.height() - viewport_hgt) / 2), int(viewport_wid), int(viewport_hgt))  
        # setWindow(x, y, w, h) sets the logical coordinates, here the middle point is (0, 0)
        qp.setWindow(int(- window_wid / 2), int(- window_hgt / 2), int(window_wid),int( window_hgt))

        self.draw_road(qp)
        # self.draw_traj(qp) # Display trajectory, for debugging
        self.draw_vehs(qp)

        ts = Simulator.getInstance().timestep
        self.mainw.step_lbl.setText("Timestep: %4d" % ts)
        self.mainw.time_lbl.setText("Elapsed time: %.1f s" % (ts * veh_dt))
        # print('ts = %d, simu_t/veh_dt = %d' % (ts, simu_t / veh_dt))
        if ts >= simu_t / veh_dt:
            self.mainw.close()
    
    def gen_draw_road(self):
        '''
        In a logical coordinate system, calculate the shape required for intersection drawing
        '''
        #lw = lane width tr = turn radius al = arm length
        x1 = self.lw * self.NSl  # x1 = 3 * lane_width
        x2 = x1 + self.tr # x2 = x1 + turn_radius
        x3 = x2 + self.al # x3 = x2 + arm_length
        y1 = self.lw * self.EWl # y1 = 3 * lane_width
        y2 = y1 + self.tr # y2 = y1 + turn_radius
        y3 = y2 + self.al 
        edge_Qlines = [
            QLineF(-x3, -y1, -x2, -y1), # 西上
            QLineF(-x3, y1, -x2, y1), # 西下
            QLineF(x3, -y1, x2, -y1), # 东上
            QLineF(x3, y1, x2, y1), # East down
            QLineF(-x1, -y3, -x1, -y2), # North left
            QLineF(x1, -y3, x1, -y2), # North right
            QLineF(-x1, y3, -x1, y2), # South left
            QLineF(x1, y3, x1, y2) # South right
        ]
        edge_Qarcs = [
            [QRectF(-x2-self.tr, -y2-self.tr, 2*self.tr, 2*self.tr), 270 * 16, 90 * 16], # Upper left
            [QRectF(x1, -y2-self.tr, 2*self.tr, 2*self.tr), 180 * 16, 90 * 16], # Upper right
            [QRectF(-x2-self.tr, y1, 2*self.tr, 2*self.tr), 0 * 16, 90 * 16], # Lower left
            [QRectF(x1, y1, 2*self.tr, 2*self.tr), 90 * 16, 90 * 16] # Lower right
        ]
        center_Qlines = [
            QLineF(-x3, 0, -x2, 0),       # 西
            QLineF(x3, 0, x2, 0),         # 东
            QLineF(0, -y3, 0, -y2),       # 北
            QLineF(0, y3, 0, y2)          # 南
        ]
        stop_Qlines = [
            QLineF(-x2, 0, -x2, y1),          # 西
            QLineF(x2, 0, x2, -y1),           # 东
            QLineF(0, -y2, -x1, -y2),         # 北
            QLineF(0, y2, x1, y2)             # 南
        ]
        lane_Qlines = []
        if self.EWl > 1:
            for i in range(self.EWl - 1):
                lane_Qlines.append(QLineF(-x3, - (i+1) * self.lw, -x2, - (i+1) * self.lw)) # 西上
                lane_Qlines.append(QLineF(-x3, (i+1) * self.lw, -x2, (i+1) * self.lw))     # 西下
                lane_Qlines.append(QLineF(x3, - (i+1) * self.lw, x2, - (i+1) * self.lw))   # 东上
                lane_Qlines.append(QLineF(x3, (i+1) * self.lw, x2, (i+1) * self.lw))       # 东下
        if self.NSl > 1:
            for i in range(self.NSl - 1):
                lane_Qlines.append(QLineF(- (i+1) * self.lw, -y3, - (i+1) * self.lw, -y2)) # 北左
                lane_Qlines.append(QLineF((i+1) * self.lw, -y3, (i+1) * self.lw, -y2))     # 北右
                lane_Qlines.append(QLineF(- (i+1) * self.lw, y3, - (i+1) * self.lw, y2))   # 南左
                lane_Qlines.append(QLineF((i+1) * self.lw, y3, (i+1) * self.lw, y2))       # 南右
        return {
            'edge_Qlines': edge_Qlines,
            'edge_Qarcs': edge_Qarcs,
            'center_Qlines': center_Qlines,
            'lane_Qlines': lane_Qlines,
            'stop_Qlines': stop_Qlines
        }

    def gen_draw_traj(self, ju_track_table):
        '''
        Based on the trajectory, generate lines and arcs for drawing.
        '''
        traj_Qlines = []
        traj_Qarcs = []
        for dir, shape_list in ju_track_table.items():
            for seg in shape_list:
                if seg[0] == 'line':
                    traj_Qlines.append(QLineF(seg[1][0], seg[1][1], seg[2][0], seg[2][1]))
                else:
                    traj_Qarcs.append([
                        QRectF(seg[3][0]-seg[4], seg[3][1]-seg[4], 2*seg[4], 2*seg[4]), 
                        min(seg[5][0], seg[5][1]) * 16, 
                        math.fabs(seg[5][0] - seg[5][1]) * 16
                    ])
        return {
            'traj_Qlines': traj_Qlines,
            'traj_Qarcs': traj_Qarcs
        }

    def draw_road(self, qp):
        # intersection edge
        qp.setPen(QPen(QColor(49, 58, 135), 0.4, Qt.SolidLine))
        for ele in self.draw_road_shape['edge_Qlines']:
            qp.drawLine(ele)
        for ele in self.draw_road_shape['edge_Qarcs']:
            qp.drawArc(ele[0], ele[1], ele[2])

        # Lane center line
        qp.setPen(QPen(QColor(242, 184, 0), 0.3, Qt.SolidLine))
        for ele in self.draw_road_shape['center_Qlines']:
            qp.drawLine(ele)

        # lane lines
        qp.setPen(QPen(QColor("white"), 0.2, Qt.SolidLine))
        for ele in self.draw_road_shape['lane_Qlines']:
            qp.drawLine(ele)

       # Stop Line
        qp.setPen(QPen(QColor(244, 82, 79), 0.4, Qt.SolidLine))
        for ele in self.draw_road_shape['stop_Qlines']:
            qp.drawLine(ele)

    def draw_traj(self, qp):
        qp.setPen(QPen(QColor('darkGrey'), 0.2, Qt.SolidLine))
        for ele in self.draw_traj_shape['traj_Qlines']:
            qp.drawLine(ele)
        for ele in self.draw_traj_shape['traj_Qarcs']:
            qp.drawArc(ele[0], ele[1], ele[2]) 
    
    def draw_vehs(self, qp):
        qp.setBrush(QColor(49, 58, 135))
        qp.setPen(QPen(QColor(255, 152, 146), 0.1, Qt.SolidLine))
        qf = qp.font()
        qf.setPointSizeF(2.5)
        qf.setFamily('Consolas')
        qp.setFont(qf)
        x1 = self.lw * self.NSl
        x2 = x1 + self.tr 
        y1 = self.lw * self.EWl
        y2 = y1 + self.tr
        for veh in Simulator.getInstance().all_veh['Nap']: 
            if veh.faultyCar:
                qp.setBrush(QColor(255, 0, 0))
            else:
                qp.setBrush(QColor(49, 58, 135))
            x = - (self.lw / 2 + self.lw * veh.inst_lane)
            y = - (y2 + (-veh.inst_x))
            rect = QRectF(x - veh.veh_wid/2, y - veh.veh_len_back, veh.veh_wid, veh.veh_len)
            qp.drawRect(rect)
            # qp.drawText(rect.bottomLeft(), str(veh._id))
        for veh in Simulator.getInstance().all_veh['Nex']: 
            if veh.faultyCar:
                qp.setBrush(QColor(255, 0, 0))
            else:
                qp.setBrush(QColor(49, 58, 135))
            x = self.lw / 2 + self.lw * veh.inst_lane
            y = - (y2 + (veh.inst_x))
            rect = QRectF(x - veh.veh_wid/2, y - veh.veh_len_front, veh.veh_wid, veh.veh_len)
            qp.drawRect(rect)
            # qp.drawText(rect.bottomLeft(), str(veh._id))
        for veh in Simulator.getInstance().all_veh['Sap']:
            if veh.faultyCar:
                qp.setBrush(QColor(255, 0, 0))
            else:
                qp.setBrush(QColor(49, 58, 135))
            x = self.lw / 2 + self.lw * veh.inst_lane
            y = y2 + (-veh.inst_x)
            rect = QRectF(x - veh.veh_wid/2, y - veh.veh_len_front, veh.veh_wid, veh.veh_len)
            qp.drawRect(rect)
            # qp.drawText(rect.bottomLeft(), str(veh._id))
        for veh in Simulator.getInstance().all_veh['Sex']:
            if veh.faultyCar:
                qp.setBrush(QColor(255, 0, 0))
            else:
                qp.setBrush(QColor(49, 58, 135))
            x = - (self.lw / 2 + self.lw * veh.inst_lane)
            y = y2 + veh.inst_x
            rect = QRectF(x - veh.veh_wid/2, y - veh.veh_len_back, veh.veh_wid, veh.veh_len)
            qp.drawRect(rect)
            # qp.drawText(rect.bottomLeft(), str(veh._id))
        for veh in Simulator.getInstance().all_veh['Wap']:
            if veh.faultyCar:
                qp.setBrush(QColor(255, 0, 0))
            else:
                qp.setBrush(QColor(49, 58, 135))
            x = - (x2 + (-veh.inst_x))
            y = self.lw / 2 + self.lw * veh.inst_lane
            rect = QRectF(x - veh.veh_len_back, y - veh.veh_wid/2, veh.veh_len, veh.veh_wid)
            qp.drawRect(rect)
            # qp.drawText(rect.bottomLeft(), str(veh._id))
        for veh in Simulator.getInstance().all_veh['Wex']:
            if veh.faultyCar:
                qp.setBrush(QColor(255, 0, 0))
            else:
                qp.setBrush(QColor(49, 58, 135))
            x = - (x2 + veh.inst_x)
            y = - (self.lw / 2 + self.lw * veh.inst_lane)
            rect = QRectF(x - veh.veh_len_front, y - veh.veh_wid/2, veh.veh_len, veh.veh_wid)
            qp.drawRect(rect)
            # qp.drawText(rect.bottomLeft(), str(veh._id))
        for veh in Simulator.getInstance().all_veh['Eap']:
            if veh.faultyCar:
                qp.setBrush(QColor(255, 0, 0))
            else:
                qp.setBrush(QColor(49, 58, 135))
            x = x2 + (-veh.inst_x)
            y = - (self.lw / 2 + self.lw * veh.inst_lane)
            rect = QRectF(x - veh.veh_len_front, y - veh.veh_wid/2, veh.veh_len, veh.veh_wid)
            qp.drawRect(rect)
            # qp.drawText(rect.bottomLeft(), str(veh._id))
        for veh in Simulator.getInstance().all_veh['Eex']:
            if veh.faultyCar:
                qp.setBrush(QColor(255, 0, 0))
            else:
                qp.setBrush(QColor(49, 58, 135))
            x = x2 + veh.inst_x
            y = self.lw / 2 + self.lw * veh.inst_lane
            rect = QRectF(x - veh.veh_len_back, y - veh.veh_wid/2, veh.veh_len, veh.veh_wid)
            qp.drawRect(rect)
            # qp.drawText(rect.bottomLeft(), str(veh._id))
        if Simulator.getInstance().rl_swap or Simulator.getInstance().evasion_swap:
            for veh in Simulator.getInstance().all_veh['ju']:
                if veh.faultyCar and veh.collidedCar:
                    qp.setBrush(QColor(255, 0, 0))
                elif veh.faultyCar:
                    qp.setBrush(QColor(0, 255, 0))
                elif veh.collidedCar:
                    qp.setBrush(QColor(0, 0, 255))
                else:
                    qp.setBrush(QColor(49, 58, 135))

                qp.save()
                qp.translate(veh.rl_x, -veh.rl_y)  # Move to the vehicle's position
                qp.rotate(- veh.heading * 180/math.pi +90)  # Apply the vehicle's rotation, negate if necessary

                # Assuming the vehicle's center of rotation is at its center
                rect = QRectF(-veh.veh_wid / 2, -veh.veh_len / 2, veh.veh_wid, veh.veh_len)
                qp.drawRect(rect)  # Draw the vehicle as a rectangle
                qp.drawText(rect.bottomLeft(), str(veh._id))  # Label the vehicle with its ID

                qp.restore()


        else: 
            for veh in Simulator.getInstance().all_veh['ju']:
                if veh.faultyCar and veh.collidedCar:
                    qp.setBrush(QColor(255, 0, 0))
                elif veh.faultyCar:
                    qp.setBrush(QColor(0, 255, 0))
                elif veh.collidedCar:
                    qp.setBrush(QColor(0, 0, 255))
                else:
                    qp.setBrush(QColor(49, 58, 135))
                # Find on which paragraph
                seg_idx = 0
                for (i, end_x) in enumerate(veh.track.ju_shape_end_x):
                    if veh.inst_x > end_x: # is greater than the end point of the i-th segment, then it is in the (i+1) segment
                        seg_idx = i + 1
                        break
                seg = veh.track.ju_track[seg_idx] #The shape of this segment
                if seg_idx > 0:
                    seg_x = veh.inst_x - veh.track.ju_shape_end_x[seg_idx - 1] # The length of this segment
                else:
                    seg_x = veh.inst_x
                if seg[0] == 'line': # It's a straight line, great
                    if abs(seg[1][0] - seg[2][0]) < 1e-5: # vertical bar
                        x = seg[1][0]
                        #print("Paint x",x)
                        if seg[1][1] < seg[2][1]: # from top to bottom
                            y = seg[1][1] + seg_x
                            #print("Paint y",y)
                            rect = QRectF(x - veh.veh_wid/2, y - veh.veh_len_back, veh.veh_wid, veh.veh_len)
                            qp.drawRect(rect)
                            qp.drawText(rect.bottomLeft(), str(veh._id))
                        else: # from bottom to top
                            y = seg[1][1] - seg_x
                            #print("Paint y",y)
                            rect = QRectF(x - veh.veh_wid/2, y - veh.veh_len_front, veh.veh_wid, veh.veh_len)
                            qp.drawRect(rect)
                            qp.drawText(rect.bottomLeft(), str(veh._id))
                    else: #Horizontal line
                        y = seg[1][1]
                        #print("Paint y",y)
                        if seg[1][0] < seg[2][0]: # from left to right
                            x = seg[1][0] + seg_x
                            #print("Paint x",x)
                            rect = QRectF(x - veh.veh_len_back, y - veh.veh_wid/2, veh.veh_len, veh.veh_wid)
                            qp.drawRect(rect)
                            qp.drawText(rect.bottomLeft(), str(veh._id))
                        else: # from right to left
                            x = seg[1][0] - seg_x
                            #print("Paint x",x)
                            rect = QRectF(x - veh.veh_len_front, y - veh.veh_wid/2, veh.veh_len, veh.veh_wid)
                            qp.drawRect(rect)
                            qp.drawText(rect.bottomLeft(), str(veh._id))
                else: # circular curve
                    qp.save()
                    #print("Paint seg,", seg)
                    qp.translate(seg[3][0], seg[3][1])
                    if seg[5][0] < seg[5][1]: # Trajectory counterclockwise
                        rotation = seg[5][0] + seg_x / seg[4] * 180 / math.pi
                        qp.rotate(- rotation) # rotate is the number of degrees clockwise
                        rect = QRectF(seg[4] - veh.veh_wid/2, - veh.veh_len_front, veh.veh_wid, veh.veh_len)
                        qp.drawRect(rect)
                        qp.drawText(rect.bottomLeft(), str(veh._id))
                    else:
                        rotation = seg[5][0] - seg_x / seg[4] * 180 / math.pi
                        qp.rotate(- rotation)
                        rect = QRectF(seg[4] - veh.veh_wid/2, - veh.veh_len_back, veh.veh_wid, veh.veh_len)
                        qp.drawRect(rect)
                        qp.drawText(rect.bottomLeft(), str(veh._id))
                    qp.restore()