;;階段のVRMLモデルを作る
;;file-name: 出力ファイル名(既に存在するファイル名を入れるとエラー)
;;step: 段数
;;length: 一段あたりの奥行き
;;width: 一段の横幅
;;height: 一段の高さ
(defun create-stair (file-name step length width height)
  (with-open-file
   (fout file-name ;;file-nameは相対パスでもよい
         :direction :output
         :if-exists :error ;;既に存在するファイルを書き換えないように
         :if-does-not-exist :create)
   (format fout "#VRML V2.0 utf8~%")
   (format fout "Transform {~%")
   (format fout "scale 1.0 1.0 1.0~%")
   (format fout "children [~%")
   (format fout "Shape {~%")
   (format fout "appearance Appearance {~%")
   (format fout "material Material {~%")
   (format fout "diffuseColor 0.1 0.1 0.1~%")  ;;色
   (format fout "specularColor 0.8 0.8 0.8~%") ;;色
   (format fout "}~%}~%")
   (format fout "geometry Extrusion {~%")
   (format fout "crossSection [0 0, ~5,5F 0, ~5,5F ~5,5F" (* step length) (* step length) (* step height))
   (dotimes (i step)
     (let ((i (1+ i))) ;;start from i=1, end with i=step
       (format fout ", ~5,5F ~5,5F" (* (- step i) length) (* (- step (1- i)) height))
       (format fout ", ~5,5F ~5,5F" (* (- step i) length) (* (- step i) height))))
   (format fout "]~%")
   ;;出現場所は原点から一段の奥行きの半分だけ前に出たところ
   (format fout "spine [~5,5F ~5,5F 0, ~5,5F ~5,5F 0]~%" (/ length 2.0) (- (/ width 2.0)) (/ length 2.0) (/ width 2.0))
   (format fout "}~%} # Shape~%]~%}")
   ))
