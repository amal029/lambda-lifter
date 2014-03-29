;;; The lambda lifter game from ICFP challenge in 2012
;;; Author: Avinash
;;; Wed Mar 26 19:33:08 NZDT 2014

(ns lambda_lifter.core
  (:gen-class))


(use 'com.phansen.clojure.adt.core)
(use '[clojure.core.match :only (match)])
(require '[clojure.java.io :as io])
(require '[clojure.contrib.str-utils2 :as s])

;;; The hamming distance function 
;;; will be used in the A* heuristic
(defn- hamming-distance [[x1 y1] [x2 y2]]
  (+ (Math/abs (- x2 x1)) (Math/abs (- y2 y1))))

(defn- print-map [mm]
  (s/map-str
   (fn [line] 
     (str (s/map-str
           #(match [%]
                   [{:robot _}] "R"
                   [{:rock  _}] "*"
                   [{:clift _}] "L"
                   [{:olift _}] "O"
                   [{:lambda _}] "\\"
                   [{:earth _}] "."
                   [{:space _}] " "
                   [{:wall _}] "#") line) "\n"))mm))

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
  (nth (filter 
        #(match [%]
                [{:robot _}] true
                [{:rock  _}] false
                [{:clift _}] false
                [{:olift _}] false
                [{:lambda _}] false
                [{:earth _}] false
                [{:space _}] false
                [{:wall _}] false) (flatten mm)) 0))

(defn- get-robot [mm]
  ;; Get the single robot!
  (:robot (get-robot-node mm)))

(defn- heuristic-cost-estimate [node1 node2]
  (let 
      [nv1  (match [node1]
                   [{:robot v}] v
                   [{:rock  v}] v
                   [{:clift v}] v
                   [{:olift v}] v
                   [{:lambda v}] v
                   [{:earth v}] v
                   [{:space v}] v
                   [{:wall v}] v
                   )
       nv2 (match [node2]
                  [{:lambda v}] v
                  [_] (Throwable. (str "The goal cannot be anything but lambda, but found " node2))
                  )
       ] (hamming-distance nv1 nv2)))

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
  (let [[u d l r] (get-vneighbors (nth (vals (get cf goal) 0)) mm M N)]
    (cond
     (= u goal) (cons :U (reconstruct-path cf u mm M N))
     (= d goal) (cons :D (reconstruct-path cf d mm M N))
     (= l goal) (cons :L (reconstruct-path cf l mm M N))
     (= r goal) (cons :R (reconstruct-path cf r mm M N))
     :else nil)))


(defn- sort-by-f [node1 node2]
  (let [nn1 (:f node1)
        nn2 (:f node2)] (compare nn1 nn2)))

;;; The A* path planning heuristic
(defn- a* [start goal mm M N]
  (let 
      [closedset (hash-set)                ; the closed map of nodes
       myhashmap (sorted-map-by sort-by-f start {:g 0 :f (heuristic-cost-estimate start goal)})
       ;; openset (sorted-set-by (sort-by-f myhashmap start start) start) ; the starting node, the robot
       came-from (hash-map)
       ]
    (loop 
        [
         cs closedset
         mhm myhashmap
         cf came-from
         ]
      ;; if the open-set is not empty then compute
      (if-not (= mhm {})
        (let 
            [current (first mhm)]
          (if (= current goal) (reconstruct-path cf goal mm M N)
              ;; this is the else part
              (let 
                  [nn (get-vneighbors (nth (vals current) 0) mm M N)]
                (loop
                    [
                     mhmm mhm
                     cff cf
                     count 0
                     neighbor (nth nn count)
                     ]
                  ;; The (:rock condition) can be loosened later on to make more interesting AI
                  (if (and (not (nil? neighbor)) (not (contains? cs neighbor)) (not (:wall neighbor)) (not (:rock neighbor)))
                    (let [tg (+ (:g (get mhmm current)) 1)
                          fn (+ tg (heuristic-cost-estimate neighbor goal))
                          ioss (not (contains? mhmm neighbor))]
                      ;; This is the internal recursion back to loop2
                      (recur (if ioss (assoc mhmm neighbor {:g tg :f fn})) 
                             (if ioss (assoc cff neighbor current)) (+ count 1) (nth nn (+ count 1)))
                      )
                    (if (< count 4) (recur mhmm cff (+ count 1) (nth nn (+ count 1))))
                    )
                  )
                ;; This is the final call back to the loop
                (recur (conj cs current) (disj mhm current) cf))))))))

(defn- get-lambda-via-ai [mm M N]
  (let 
      [
       lambdas (filter #(match [%] [{:lambda _}] %) (flatten mm))
       sorted-lambdas (sort #(heuristic-cost-estimate (get-robot-node mm) %) lambdas)
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

(defn- get-movement [ss]
  (cond
   (= (.toUpperCase ss) "L") :L
   (= (.toUpperCase ss) "R") :R
   (= (.toUpperCase ss) "U") :U
   (= (.toUpperCase ss) "D") :D
   (= (.toUpperCase ss) "A") :A
   :else nil))

(defn- play [mm M N]
  (do 
    ;; first print the map for the player
    (print (print-map mm))
    ;; Then get the input from the player
    (println "Please input a movement: ")
    (println "Valid movements are: ")
    (println "L/l: Move Robot (R) Left")
    (println "R/r: Move Robot (R) Right")
    (println "U/u: Move Robot (R) Up")
    (println "D/d: Move Robot (R) Down ")
    (println "A/a: Collect 1-lambda automatically ")
    (print "> ")
    (flush)
    (let 
        [
         movement (get-movement (read-line))
         ]
      (cond
       (and (not (nil? movement)) (not (= movement :A))) (recur (update-map (move-robot movement mm M N) M N) M N)
       (and (not (nil? movement)) (= movement :A)) 
       (let 
           [mmm (ref mm)
            movements (get-lambda-via-ai mm M N)
            ]
         (map #(do (dosync (ref-set mmm (move-robot % @mmm M N)) (print-map @mmm) (Thread/sleep 600))) movements)
         (recur @mmm M N))
       :else (recur mm M N)
       ))))

(defn -main [& args]
  "the main function that plays the game"
  (let 
      [
       [M N mm] (consume-map (slurp (nth args 0))) 
       ]
    (do (play mm M N) (flush))))
