
#########################
### ColorPowerPlanner ###
#########################
class ColorPowerPlanner():
    """
    指定した範囲の色の物体の座標を取得する関数
    frame: 画像
    AREA_RATIO_THRESHOLD: area_ratio未満の塊は無視する
    LOW_COLOR: 抽出する色の下限(h,s,v)
    HIGH_COLOR: 抽出する色の上限(h,s,v)
    
    色の設定
    0 <= h <= 179 (色相)　OpenCVではmax=179なのでR:0(180),G:60,B:120となる
    0 <= s <= 255 (彩度)　黒や白の値が抽出されるときはこの閾値を大きくする
    0 <= v <= 255 (明度)　これが大きいと明るく，小さいと暗い
    """
    
    # Coefficient between ewbsite and numpy
    hsv_coef = np.array([1/2, 2.55, 2.55])
    
    # 速度の設定
    STANDARD_POWER = 65
    POWER_RANGE = 15

    #{1:red,0:blue,99:orange}
    # h:0~360, s:0~100, v:0~100
    
    ##orange
    LOW_COLOR_EDIT = {1:np.array([300, 59, 45]),0:np.array([200, 40, 70]),99:np.array([41, 60, 60])}
    HIGH_COLOR_EDIT = {1:np.array([360, 100, 100]),0:np.array([250, 100, 100]),99:np.array([43, 90, 100])}
    
    ##purple
    #LOW_COLOR_EDIT = {1:np.array([300, 59, 45]),0:np.array([200, 40, 70]),99:np.array([,,])}
    #HIGH_COLOR_EDIT = {1:np.array([360, 100, 100]),0:np.array([250, 100, 100]),99:np.array([,,)}
    

    # 抽出する色の塊のしきい値
    AREA_RATIO_THRESHOLD = 0.00003
    def __init__(self):
        self.pos = []
        ## DO NOT TOUCH HERE
        # h:1~179, s:1~255, v:1~255
        self.LOW_COLOR = {k:np.round(self.LOW_COLOR_EDIT[k]*self.hsv_coef) for k in self.LOW_COLOR_EDIT.keys()}
        self.HIGH_COLOR = {k:np.round(self.HIGH_COLOR_EDIT[k]*self.hsv_coef) for k in self.HIGH_COLOR_EDIT.keys()}
    

    def find_specific_color(self,frame,AREA_RATIO_THRESHOLD,LOW_COLOR,HIGH_COLOR,connecting_state):
        # 高さ，幅，チャンネル数
        h,w,c = frame.shape

        # hsv色空間に変換
        hsv = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
        
        # 色を抽出する
        ex_img = cv2.inRange(hsv,LOW_COLOR[connecting_state],HIGH_COLOR[connecting_state])

        # 輪郭抽出
        # 変更点 < opencvのバージョンの違いにより？引数を少なく設定>
        #_,contours,hierarchy = cv2.findContours(ex_img,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        contours,hierarchy = cv2.findContours(ex_img,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        # 面積を計算
        areas = np.array(list(map(cv2.contourArea,contours)))
        
        if len(areas) == 0 or np.max(areas) / (h*w) < AREA_RATIO_THRESHOLD:
            # 見つからなかったらNoneを返す
            try:
                print(np.max(areas) / (h*w) )
            except:
                print("no color")
            return None
        else:
            #print("@powerplanner\ncolor area = ",np.max(areas) / (h*w))
            # 面積が最大の塊の重心を計算し返す
            max_idx = np.argmax(areas)
            max_area = areas[max_idx]
            max_a = areas[max_idx]
            result = cv2.moments(contours[max_idx])
            x = int(result["m10"]/result["m00"])
            y = int(result["m01"]/result["m00"])
            return (x,y,max_area)

    def power_calculation(self,pos,h,w,flag):
        if not flag:
            xn = 2*(pos[0]+300-w/2) / w + 0.00000001 ### + 300 ireru no kottijanai??
            power_R = int(self.STANDARD_POWER - self.POWER_RANGE * xn)
            power_L = int(self.STANDARD_POWER + self.POWER_RANGE * xn+5)
        else:
            xn = 2*(pos[0]-w/2) / w + 0.00000001
            power_R = -int(xn/abs(xn)*(self.STANDARD_POWER*1.15 + self.POWER_RANGE * abs(xn))) ### +- ga umareru youni
            power_L = -power_R+int(xn/abs(xn))*7
        w_rate = abs(xn) ### sleep zikan keisan you
        return power_R,power_L,w_rate

    def power_planner(self,frame,connecting_state,ar_count=0):
        """
        arg:
            frame
        return:
            {"R":power_R,"L":power_L,"Clear":bool} 
        """
        move = 'stop'
        height, width = frame.shape[:2]

        aprc_clear = False #これは目標に到達できたかのbool値

        self.pos = self.find_specific_color(
                frame,
                self.AREA_RATIO_THRESHOLD,
                self.LOW_COLOR,
                self.HIGH_COLOR,
                connecting_state
            )
        
        if self.pos is not None:
            detected = True
            print(self.pos[2])
            if connecting_state == 0:
                if self.pos[2] > 7500:   #2000 datta yo
                    aprc_clear = True #これは目標に到達できたかのbool値
            else:
                #print(self.pos[2])
                if self.pos[2] > 12000:
                # arm temae : 28000
                # arm red : 25000
                    aprc_clear = True #これは目標に到達できたかのbool値
            if ar_count > 0:
                aprc_clear = True
            print("aprc_clear : ",aprc_clear)
            power_R, power_L, w_rate = self.power_calculation(self.pos,height,width,aprc_clear)
            
        else:
            self.pos = ["none","none","none"]
            move = 'stop'
            power_R, power_L = 0,0
            w_rate = None ### mienai toki ni None ni naruyouni
            detected = False
        return {"R":power_R,"L":power_L,"Clear":aprc_clear,"Detected_tf":detected,"w_rate":w_rate,"move":move} ### sleep zikan keisan ni motiiru node w_rate wo dasu

    def para_detection(self,frame):
        height, width = frame.shape[:2]

        self.pos = self.find_specific_color(
                frame,
                self.AREA_RATIO_THRESHOLD,
                self.LOW_COLOR,
                self.HIGH_COLOR,
                99
            )
        
        aprc_clear = True
        move = 'stop'
        
        if self.pos is not None:
            if self.pos[2] > 6000:
                detected = True
                power_L, power_R, w_rate = self.power_calculation(self.pos,height,width,aprc_clear)
                if power_L > power_R:
                    move = 'stay-right'
                else:
                    move = 'stay-left'
            else:
                move = 'stop'
                power_R, power_L = 0,0
                w_rate = None ### mienai toki ni None ni naruyouni
                detected = False
        else:
            self.pos = ["none","none","none"]
            move = 'stop'
            print("here")
            power_R, power_L = 0,0
            w_rate = None ### mienai toki ni None ni naruyouni
            detected = False

        return {"R":power_R,"L":power_L,"Clear":aprc_clear,"Detected_tf":detected,"w_rate":w_rate,"move":move} ### sleep zikan keisan ni motiiru node w_rate wo dasu

