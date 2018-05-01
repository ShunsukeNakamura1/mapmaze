//�n�}������H�����܂�
//�N���X���g�p
//���܂������Ȃ�����

#include "DxLib.h"
#define MAPSIZE 500 //�ő�}�b�v�T�C�Y

class Map{
	enum{
		ROAD = 0, //��
		WALL = 1, //��
	};

private:
	char map[MAPSIZE][MAPSIZE];
	int height, width;
	unsigned int white,  black;

public:
	/*�R���X�g���N�^
	�ǂݍ��݉摜�̖��O�������Ƃ���*/
	Map(char* name){
		int handle, r, g, b, a;
		handle = LoadSoftImage(name);
		GetSoftImageSize( handle, &width, &height );
		for(int i = 0; i < height && i < MAPSIZE; i ++ ){
			for(int j = 0; j < width && j < MAPSIZE; j ++ ){
				// �P�h�b�g�̐F���擾
				GetPixelSoftImage( handle, j, i, &r, &g, &b, &a ) ;
				if(r >= 230 && g >= 230 && b >= 230){
					map[j][i] = ROAD;
				}else{
					map[j][i] = WALL;
				}
			}
		}
		white = GetColor(255, 255, 255);
		black = GetColor(  0,   0,   0);
		DeleteSoftImage( handle );
	}

	//�n�}�̕`��
	void drawMap(int x_gap, int y_gap){
		for(int i = 0; i < height && i < MAPSIZE; i ++ ){
			for(int j = 0; j < width && j < MAPSIZE; j ++ ){
	            if(map[j][i] == ROAD){
					DrawBox(j, i, j + 1, i + 1, white, TRUE);
				}else if(map[j][i] == WALL){
					
				}

			}
		}
	}
};

int WINAPI WinMain( HINSTANCE hInstance, HINSTANCE hPrevInstance, LPSTR lpCmdLine, int nCmdShow )
{
	// �E�C���h�E���[�h�ɕύX
    ChangeWindowMode( TRUE );

    // �c�w���C�u�����̏�����
    if( DxLib_Init() < 0 ){
		return -1;
	}
    // �}�b�v�N���X������
    Map map("map1.PNG");

    //�`��
	map.drawMap(0, 0);

    // �L�[���͑҂�
    WaitKey();

    // �c�w���C�u�����̌�n��
    DxLib_End();

    // �\�t�g�̏I��
    return 0;
}