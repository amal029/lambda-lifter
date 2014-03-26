;;; The lambda lifter game from ICFP challenge in 2012
;;; Author: Avinash
;;; Wed Mar 26 19:33:08 NZDT 2014

;;; TODO: add the movement function with the constraints
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
                   [ {:rock  _}] "*"
                   [ {:clift _}] "L"
                   [ {:olift _}] "O"
                   [ {:lambda _}] "\\"
                   [ {:earth _}] "."
                   [ {:space _}] " "
                   [ {:wall _}] "#") line) "\n"))mm))

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
                        [ {:rock  _}] false
                        [ {:clift _}] false
                        [ {:olift _}] false
                        [ {:lambda _}] false
                        [ {:earth _}] false
                        [ {:space _}] false
                        [ {:wall _}] false) (flatten mm)) 0)))

(defn- get-neighbors [i j mm M N]
  (let [vmm (mapv identity (flatten mm))]
    [(cond 
      (>= (- i 1) 0) (vmm (+ (* (- i 1) M) j)) ;this is up 
      :else nil)
     (cond 
      (<= (+ i 1) (- M 1)) (vmm (+ (* (+ i 1) M) j)) ;this is down 
      :else nil) 
     (cond 
      (>= (- j 1) 0) (vmm (+ (* i M) (- j 1))) ;this is left 
      :else nil) 
     (cond 
      (<= (+ j 1) (- M 1)) (vmm (+ (* i M) (+ j 1))) ;this is right
      :else nil)
     ]))

(defn -main [& args]
  "the main function that plays the game"
  (let 
      [
       [M N mm] (consume-map (slurp (nth args 0))) 
       ;; get the robots position
       [ri rj] (get-robot mm)
       ;; get value from position
       [l r u d] (get-neighbors ri rj mm M N)
       ]
    (print (print-map mm))
    (print [l r u d])))
