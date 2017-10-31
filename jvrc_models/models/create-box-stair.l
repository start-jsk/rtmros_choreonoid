;;箱を並べて階段のVRMLモデルを作る
;;file-name: 出力ファイル名(既に存在するファイル名を入れるとエラー)
;;           config等で指定しやすいようにkey引数としている。作り直す場合は一度ファイル消してから
;;step: 段数
;;length: 一段あたりの奥行き[m]
;;width: 一段の横幅[m]
;;height: 一段の高さ[m]
(defun create-box-stair (step length width height &key (file-name "box-stair.wrl"))
  (with-open-file
   (fout file-name ;;file-nameは相対パスでもよい
         :direction :output
         :if-exists :error ;;既に存在するファイルを書き換えないように
         :if-does-not-exist :create)
   (format fout "#VRML V2.0 utf8~%")
   (dotimes (i step)
     (let ((i (1+ i))) ;;start from i=1
       (dotimes (j i)
         (let ((j (1+ j))) ;start from j=1
           (format fout "Transform {~%")
           (format fout "translation ~5,5F 0 ~5,5F~%" (* length i) (+ (/ height 2.0) (* height (1- j))))
           (format fout "scale 1.0 1.0 1.0~%")
           (format fout "children [~%")
           (format fout "Shape {~%")
           (format fout "appearance Appearance {~%")
           (format fout "material Material {~%")
           (format fout "diffuseColor 0.1 0.1 0.1~%")  ;;色
           (format fout "specularColor 0.8 0.8 0.8~%") ;;色
           (format fout "}~%}~%")
           (format fout "geometry Box {~%")
           (format fout "size ~5,5F ~5,5F ~5,5F~%" length width height)
           (format fout "}~%")
           (format fout "} #Shape~%")
           (format fout "]~%")
           (format fout "}~%")
           (format fout "~%")
           ))  ;;end-j
       ))  ;;end-i
   ))
