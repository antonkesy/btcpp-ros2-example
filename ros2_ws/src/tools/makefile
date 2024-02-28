GROOT_BIN="./Groot/build/Groot"

build_groot: 
	cd Groot && \
	mkdir build -p && cd build && \
	cmake .. && \
	make 

groot_bringup: build_groot
	${GROOT_BIN} --mode editor &

clean:
	sudo rm -rf Groot/build
