SOURCE := video_to_pic.cpp
TARGET := video_to_pic

CC := g++
LIBS := `pkg-config opencv --cflags --libs`

$(TARGET) : $(SOURCE)
	$(CC) $(SOURCE) -o $(TARGET) $(LIBS)
