def findobj1(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray,(1,1),0)
    v2 = np.median(blur)
    lower = int(max(0, (1.0 - 0.0001) * v2))
    upper = int(min(255, (1.0 + 0.0001) * v2))
    bordas = cv2.Canny(blur, lower, upper)    
    cv2.imwrite("test.jpg",frame)
    img1 = cv2.imread('pano2.jpg',0)          # Imagem a procurar
    img2 = cv2.imread("test.jpg",0)
    findCircle = False
    findFox =False
    circles = []
    bordas_color = cv2.cvtColor(bordas, cv2.COLOR_GRAY2BGR)
    circles = None
    circles=cv2.HoughCircles(bordas,cv2.HOUGH_GRADIENT,2,40,param1=5,param2=150,minRadius=1,maxRadius=60)
    if circles is not None:
        circles = np.uint16(np.around(circles))
        for i in circles[0,:]:   
            findCircle = True
            cv2.circle(frame,(i[0],i[1]),i[2],(0,255,0),2)
            cv2.circle(frame,(i[0],i[1]),2,(0,0,255),3)
    MIN_MATCH_COUNT = 10
    if findCircle == True:
        sift = cv2.xfeatures2d.SIFT_create()
        kp1, des1 = sift.detectAndCompute(img1,None)
        kp2, des2 = sift.detectAndCompute(img2,None)
        FLANN_INDEX_KDTREE = 0
        index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
        search_params = dict(checks = 50)
        flann = cv2.FlannBasedMatcher(index_params, search_params)
        matches = flann.knnMatch(des1,des2,k=2)
        good = []
        for m,n in matches:
            if m.distance < 0.7*n.distance:
                good.append(m)
        if len(good)>MIN_MATCH_COUNT:
            findFox = True
            src_pts = np.float32([ kp1[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
            dst_pts = np.float32([ kp2[m.trainIdx].pt for m in good ]).reshape(-1,1,2)
            M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,5.0)
            matchesMask = mask.ravel().tolist()
            h,w = img1.shape
            pts = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2)
            dst = cv2.perspectiveTransform(pts,M)
            img2b = cv2.polylines(frame,[np.int32(dst)],True,255,3, cv2.LINE_AA)
    sift = cv2.xfeatures2d.SIFT_create()
    kpts = sift.detect(frame)
    x = [k.pt[0] for k in kpts]
    y = [k.pt[1] for k in kpts]
    s = [(k.size/2)**2 * pi for k in kpts]
    plt.scatter(x, y, s, c='r', alpha=0.4)
    plt.imshow(frame, cmap=cm.gray)
    plt.title('SIFT')
    if findFox == True:
        return(True)
    else:
        return(False)