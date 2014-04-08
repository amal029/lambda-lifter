;;; The lambda lifter game from ICFP challenge in 2012
;;; Author: Avinash
;;; Wed Mar 26 19:33:08 NZDT 2014

(ns lambda_lifter.core
  (:gen-class))

(use 'com.phansen.clojure.adt.core)
(use '[clojure.core.match :only (match)])
(require '[clojure.java.io :as io])
(require '[clojure.contrib.str-utils2 :as s])
(require '[clojure.core.typed :as ct])
(use 'seesaw.core)
(use 'seesaw.graphics)
(import 'java.awt.image.BufferedImage)

;;; native look and feel
(native!)

;;; Some global definitions
;;; These need to be converted to relative paths
(def robot (javax.imageio.ImageIO/read (java.io.File. "images/robot.jpg")))
(def rock (javax.imageio.ImageIO/read (java.io.File. "images/rock.jpg")))
(def clift (javax.imageio.ImageIO/read (java.io.File. "images/clift.jpg")))
(def olift (javax.imageio.ImageIO/read (java.io.File. "images/olift.jpg")))
(def lambda (javax.imageio.ImageIO/read (java.io.File. "images/lambda.jpg")))
(def space (javax.imageio.ImageIO/read (java.io.File. "images/space.jpg")))
(def wall (javax.imageio.ImageIO/read (java.io.File. "images/wall.jpg")))
(def earth (javax.imageio.ImageIO/read (java.io.File. "images/earth.jpg")))

(def image-width (.getWidth wall))
(def image-height (.getHeight wall))

(def inter-map (ref nil))

(defn paint-map [g]
  (doall 
   (map
    (fn [line] 
      (doall (map
              #(match [%]
                      [{:robot [x y]}] (.drawImage g robot (* y image-width) (* x image-height) nil)
                      [{:rock  [x y]}] (.drawImage g rock (* y image-width) (* x image-height) nil)
                      [{:clift [x y]}] (.drawImage g clift (* y image-width) (* x image-height) nil)
                      [{:olift [x y]}] (.drawImage g olift (* y image-width) (* x image-height) nil)
                      [{:lambda [x y]}] (.drawImage g lambda (* y image-width) (* x image-height) nil)
                      [{:earth [x y]}] (.drawImage g earth (* y image-width) (* x image-height) nil)
                      [{:space [x y]}] (.drawImage g space (* y image-width) (* x image-height) nil)
                      [{:wall [x y]}] (.drawImage g wall (* y image-width) (* x image-height) nil))line)))@inter-map)))

;;; The hamming distance function 
;;; will be used in the A* heuristic
(defn- hamming-distance [[x1 y1] [x2 y2]]
  (+ (Math/abs (- x2 x1)) (Math/abs (- y2 y1))))

(defn- consume-map [map]
  (let 
      [lines (clojure.string/split-lines map)
       M (count lines)
       N (count (nth lines 0))]
    [M N (map-indexed 
          (fn [i l] 
            (map-indexed (fn [j k]
                           (cond 
                            (= "R" (str k)) {:robot [i j]}
                            (= "*" (str k)) {:rock [i j]}
                            (= "L" (str k)) {:clift [i j]}
                            (= "O" (str k)) {:olift [i j]}
                            (= "\\" (str k)) {:lambda [i j]}
                            (= "#" (str k)) {:wall [i j]}
                            (= "." (str k)) {:earth [i j]}
                            (= " " (str k)) {:space [i j]}
                            :else (throw (Throwable. (str "undefined character in the map: "  k)))
                            ))l)) lines)]))

(defn- get-robot-node [mm]
  (first (filter 
          #(match [%]
                  [{:robot _}] true
                  [_] false) (flatten mm))))

(defn- get-robot [mm]
  ;; Get the single robot!
  (:robot (get-robot-node mm)))

(defn- heuristic-cost-estimate [node1 node2]
  ;; Change this call when doing a different type of map
  (hamming-distance (first (vals node1)) (first (vals node2))))

(defn- empty-earth-lambda-olambda? [pos]
  (do 
    (match [pos]
           [{:space _}] true
           [{:earth _}] true
           [{:lambda _}] true
           [{:olift _}] true
           [_] false)))

(defn- get-neighbors [i j mm M N]
  (let [vmm (mapv identity (flatten mm))
        ]
    [(cond 
      (>= (- i 1) 0) (vmm (+ (* (- i 1) N) j)) ;this is up 
      :else nil)
     (cond 
      (<= (+ i 1) (- M 1)) (vmm (+ (* (+ i 1) N) j)) ;this is down 
      :else nil) 
     (cond 
      (>= (- j 1) 0) (vmm (+ (* i N) (- j 1))) ;this is left 
      :else nil) 
     (cond 
      (<= (+ j 1) (- N 1)) (vmm (+ (* i N) (+ j 1))) ;this is right
      :else nil)
     ]))

(defn- get-vneighbors [[i j] mm M N]
  (get-neighbors i j mm M N)) 

;;; The path reconstruction after a*
(defn- reconstruct-path [cf goal mm M N]
  (let [
        ;; This is crappy code, but I am tired
        [i j] (nth (vals (get cf goal)) 0)
        [k l] (if (and (nil? i) (nil? j)) [0 0] [i j])
        [u d l r] (get-neighbors k l mm M N)]
    (cond
     (= u goal) (cons :U (reconstruct-path cf (get cf goal) mm M N))
     (= d goal) (cons :D (reconstruct-path cf (get cf goal) mm M N))
     (= l goal) (cons :L (reconstruct-path cf (get cf goal) mm M N))
     (= r goal) (cons :R (reconstruct-path cf (get cf goal) mm M N))
     :else nil)))

(defn- get-lowest-f [hm]
  ((first (sort-by (fn [[x y]] x) (map (fn [x y] [(:f y) x]) (keys hm) (vals hm))))1))

;;; The A* path planning heuristic
(defn- a* [start goal mm M N]
  (loop 
      [
       cs (hash-set)             ; the closed map of nodes
       mhm (hash-map start {:g 0 :f (heuristic-cost-estimate start goal)})
       cf (hash-map)
       ]
    ;; if the open-set is not empty then compute
    (if-not (= mhm {})
      (let 
          ;; this just is the key!!
          [current (get-lowest-f mhm)]
        (if (= current goal) (reconstruct-path cf goal mm M N)
            ;; this is the else part
            (let 
                [nn (get-vneighbors (first (vals current)) mm M N)
                 [vmhmm vcff] (reduce 
                               (fn [[mhmm cff] neighbor]
                                 (if (and (not (nil? neighbor)) (not (contains? cs neighbor)) (not (:wall neighbor)) (not (:rock neighbor)))
                                   (let [tg (+ (:g (get mhmm current)) 1)
                                         fn (+ tg (heuristic-cost-estimate neighbor goal))]
                                     (cond (nil? (get mhmm neighbor)) [(assoc mhmm neighbor {:g tg :f fn}) (assoc cff neighbor current)]
                                           :else [mhmm cff]))
                                   [mhmm cff])) [mhm cf] nn)]
              ;; This is the final call back to the loop
              (recur (conj cs current) (dissoc vmhmm current) vcff)))))))

(defn- get-lambda-via-ai [mm M N]
  (let 
      [
       lambdas (filter #(match [%] [{:lambda _}] true [_] false) (flatten mm))
       sorted-lambdas (sort-by #(heuristic-cost-estimate (get-robot-node mm) %) lambdas)
       ]
    (a* (get-robot-node mm) (nth sorted-lambdas 0) mm M N)
    ))


(defn- replace-if-necessary [replacements index eres N]
  (let 
      [res (filter #(match [%]
                           [{:robot [x y]}] (= index (+(* x N)y))
                           [{:space [x y]}] (= index (+(* x N)y))
                           [{:rock [x y]}]  (= index (+(* x N)y))
                           [_] false
                           ) replacements)]
    (if (and (not (nil? res)) (= (count res) 1)) (nth res 0)
        eres
        )))

(defn- update-map [mm M N]
  (let
      [m (atom nil)]
    (map
     (fn [l]
       (map
        (fn [c]
          (match [c]
                 [{:rock [x y]}] (let [[u d l r ] (get-neighbors x y mm M N)
                                       [_ _ _ rr ] (get-neighbors (+ x 1) y mm M N)
                                       ] 
                                   (cond 
                                    (not (nil? (:space d))) (do (swap! m conj {:rock [(+ x 1) y]}) {:space [x y]})
                                    (and (not (nil? (:empty rr))) (not (nil? (:lambda r)))) (do (swap! m conj {:rock [(+ x 1) (+ y 1)]}) {:space [x y]})
                                    :else c
                                    ))
                 [{:space [x y]}] (let [pp (replace-if-necessary @m (+ (* x N) y) c N)] (if (not (nil? pp)) pp c))
                 [_] c))l))mm)))

(defn- move-robot-on-map [mm M N _ replacements] 
  (map-indexed
   (fn [i l]
     (map-indexed
      (fn [j k] (replace-if-necessary replacements (+(* i N) j) k N))l))mm))

(defn- move-robot [command mm M N]
  (let [
        [ri rj] (get-robot mm)
        [u d l r] (get-neighbors ri rj mm M N)
        ]
    (match [command]
           [:L] (cond 
                 (empty-earth-lambda-olambda? l) (do (move-robot-on-map mm M N :L [{:space [ri rj]} {:robot (nth (vals l) 0)}]))
                 (not (nil? (:rock l)))
                 (let [[_ _ lr _] (get-vneighbors (:rock l) mm M N)]
                   (if (and (not (nil? lr)) (not (nil? (:space lr))))
                     (move-robot-on-map mm M N :L [{:space [ri rj]} {:robot (nth (vals l) 0)}  {:rock (nth (vals lr) 0)}])
                     mm))
                 :else mm)
           [:R] (cond 
                 (empty-earth-lambda-olambda? r) (move-robot-on-map mm M N :R [{:space [ri rj]} {:robot (nth (vals r) 0)}])
                 (not (nil? (:rock r)))
                 (let [[_ _ _ rr] (get-vneighbors (:rock r) mm M N)]
                   (if (and (not (nil? rr)) (not (nil? (:space rr))))
                     (move-robot-on-map mm M N :R [{:space [ri rj]} {:robot (nth (vals r) 0)} {:rock (nth (vals rr) 0)}])
                     mm))
                 :else mm)
           [:U] (cond 
                 (empty-earth-lambda-olambda? u) (move-robot-on-map mm M N :U [{:space [ri rj]} {:robot (nth (vals u) 0)}])
                 :else mm)
           [:D] (cond 
                 (empty-earth-lambda-olambda? d) (move-robot-on-map mm M N :D [{:space [ri rj]} {:robot (nth (vals d) 0)}])
                 :else mm)
           )))

(defn- play [M N movement]
  (do 
    (cond
     (and (not (nil? movement)) (not (= movement :A))) (dosync (ref-set inter-map (update-map (move-robot movement @inter-map M N) M N)))
     (and (not (nil? movement)) (= movement :A)) 
     (let 
         [mmm (atom @inter-map)
          movements (reverse (get-lambda-via-ai @inter-map M N))
          ]
       (doall (map #(reset! mmm (update-map (move-robot % @mmm M N) M N)) movements))
       (dosync (ref-set inter-map @mmm))))))

(defn -main [& args]
  "the main function that plays the game"
  (let 
      [[M N mm] (consume-map (slurp (nth args 0)))
       _ (dosync (ref-set inter-map mm))
       c (canvas :background "#BBBBDD" :visible? true :id :cc :paint  #(paint-map %2))
       f (frame  :title "Lambda lifter" :resizable? false 
                 :width (* N image-width) :height (* M image-height) 
                 :on-close :exit
                 :content (border-panel :id :b :border 5 :hgap 5 :vgap 5 
                                        :center c))
       _ (listen f :key-typed #(let 
                                   [k (.getKeyChar %)
                                    movement (cond 
                                              (= k \a) :A
                                              (= k \l) :L
                                              (= k \u) :U
                                              (= k \d) :D
                                              (= k \r) :R
                                              :else nil)] 
                                 (play M N movement) 
                                 (repaint! c)))]
    (show! f) (flush)))
