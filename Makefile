dependency:
	cd build && cmake .. --graphviz=dependency.dot && dot -Tpng dependency.dot -o dependency.png


prepare:
	rm -rf build
	mkdir build
	# ggf die beiden Befehle um conan zu starten
	#conan config init
	#source ~/.profile
	cd build && conan install .. && cd ..
