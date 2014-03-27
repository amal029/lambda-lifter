;;; The lambda lifter game from ICFP challenge in 2012
;;; Author: Avinash
;;; Wed Mar 26 19:33:08 NZDT 2014

;;; TODO: add the AI that plays the game

(ns lambda_lifter.core
  (:gen-class))


(use 'com.phansen.clojure.adt.core)
(use '[clojure.core.match :only (match)])
(require '[clojure.java.io :as io])
(require '[clojure.contrib.str-utils2 :as s])


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

(defn- get-robot [mm]
  ;; Get the single robot!
  (:robot (nth (filter 
                #(match [%]
                        [{:robot _}] true
                        [{:rock  _}] false
                        [{:clift _}] false
                        [{:olift _}] false
                        [{:lambda _}] false
                        [{:earth _}] false
                        [{:space _}] false
                        [{:wall _}] false) (flatten mm)) 0)))

(defn- empty-earth-lambda-olambda? [pos]
  (do 
    (println pos)
    (match [pos]
          [{:space _}] true
          [{:earth _}] true
          [{:lambda _}] true
          [{:olift _}] true
          [_] false)))

(defn- get-neighbors [i j mm M N]
  (let [vmm (mapv identity (flatten mm))
        ;; _ (println vmm)
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

;;; Need an atom or mutable type
;;; Will complete map-update later
;;; Just use the java array
(defn- update-map [mm M N]
  (map-indexed 
   (fn [i l]
     (map-indexed
      (fn [j c]
        (match [c]
               [{:rock [x y]}] (let [[_ d _ _ ] (get-vneighbors x y mm M N)] (if (not (nil? (:empty d))) mm))
               ))l))mm))

(defn- move-robot-on-map [mm M N _ replacements] 
  (map-indexed
   (fn [i l]
     (map-indexed
      (fn [j k] (replace-if-necessary replacements (+(* i N) j) k N))l))mm))

(defn- move-robot [command mm M N]
  (let [
        [ri rj] (get-robot mm)
        [u d l r] (get-neighbors ri rj mm M N)
        _ (println "robot: " ri rj)
        _ (println "command: " command " neighbors: " u d l r)
        ]
    (match [command]
           [:L] (cond 
                 (empty-earth-lambda-olambda? l) (do (println l) (move-robot-on-map mm M N :L [{:space [ri rj]} {:robot (nth (vals l) 0)}]))
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
    (print "> ")
    (flush)
    (let 
        [
         movement (get-movement (read-line))
         ]
      (cond
       (not (nil? movement)) (recur (move-robot movement mm M N) M N)
       :else (recur mm M N)
       ))))

(defn -main [& args]
  "the main function that plays the game"
  (let 
      [
       [M N mm] (consume-map (slurp (nth args 0))) 
       ]
    (do (play mm M N) (flush))))
