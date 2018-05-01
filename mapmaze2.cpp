//�n�}������H�����܁[��
//�X�^�[�g�E�S�[��������

#include "DxLib.h"
#include <stack>
#include <vector>

#define FILENAME "small1.PNG" //�ǂݍ��ރt�@�C���̖��O
//#define FILENAME "a.png" //�ǂݍ��ރt�@�C���̖��O
//#define FILENAME "map4.PNG" //�ǂݍ��ރt�@�C���̖��O
//#define FILENAME "b.png" //�ǂݍ��ރt�@�C���̖��O �ȒP�Ȃ��
#define MAPSIZE 1500//�ő�}�b�v�T�C�Y
#define WALL 0      //��
#define ROAD 1      //��
#define VERTEX 2    //���_
#define WINDOWX 1000//�E�B���h�E�T�C�Y
#define WINDOWY 800
#define SMALL 500   //���x���t�����ꂽ�̈�̖ʐς�SMALL�ȉ����Ə��̈�Ƃ���
#define CUTCOUNT 200//���̐ڑ���؂��

//���[���[����
struct tMouse{ //�}�E�X�C�x���g�̂��߂̍\����
	int x;		//x�����̂���
	int y;		//y�����̂���
	int size;	//�}�b�v�̃T�C�Y
};
struct XY{
	int x;
	int y;
};
struct tVer{  //���_
	tVer(){}
	tVer(int x_, int y_){
		x = x_;
		y = y_;
	}
	tVer(int number, int x_, int y_){
		num = number;
		x = x_;
		y = y_;
	}
	int num;  //���_�ԍ�
	int x;    //x���W
	int y;    //y���W
};
struct tEdge{  //��
	tEdge(){}
	tEdge(tVer s, tVer d,int cost_, int angle_){
		src = s;
		dst = d;
		cost = cost_;
		angle = angle_;
	}
	tVer src;  //���_1
	tVer dst;  //���_2
	int cost;
	//�ӒT���p(���_src���璸�_dst���ǂ̕����ɂ��邩)
	//  2
	//1 ! 3
	//  0
	int angle; 
	
};
/*  ���j�I���t�@�C���h �Q�l:http://dai1741.github.io/maximum-algo-2012/docs/minimum-spanning-tree/  */
struct UnionFind{
	// par[i]�F�f�[�^i��������؂̐e�̔ԍ��Bi == par[i]�̂Ƃ��A�f�[�^i�͖؂̍��m�[�h�ł���
	std::vector<int> par;
	// sizes[i]�F���m�[�hi�̖؂Ɋ܂܂��f�[�^�̐��Bi�����m�[�h�łȂ��ꍇ�͖��Ӗ��Ȓl�ƂȂ�
	std::vector<int> sizes;

	UnionFind(int n) : par(n), sizes(n, 1) {
		// �ŏ��͑S�Ẵf�[�^i���O���[�vi�ɑ��݂�����̂Ƃ��ď�����
		for (int i=0;i < n;i++){
			par[i] = i;
		}
	}

	// �f�[�^x��������؂̍��𓾂�
	int find(int x) {
		if (x == par[x]) return x;
		return par[x] = find(par[x]);  // ���𒣂�ւ��Ȃ���ċA�I�ɍ��m�[�h��T��
	}

	// 2�̃f�[�^x, y��������؂��}�[�W����
	void unite(int x, int y) {
		// �f�[�^�̍��m�[�h�𓾂�
		x = find(x);
		y = find(y);

		// ���ɓ����؂ɑ����Ă���Ȃ�}�[�W���Ȃ�
		if(x == y){
			return;
		}

		// x�̖؂�y�̖؂��傫���Ȃ�悤�ɂ���
		if (sizes[x] < sizes[y]) std::swap(x, y);

		// x��y�̐e�ɂȂ�悤�ɘA������
		par[y] = x;
		sizes[x] += sizes[y];
		// sizes[y] = 0;  // sizes[y]�͖��Ӗ��Ȓl�ƂȂ�̂�0�����Ă����Ă��悢
	}

	// 2�̃f�[�^x, y��������؂������Ȃ�true��Ԃ�
	bool same(int x, int y) {
		return find(x) == find(y);
	}

	// �f�[�^x���܂܂��؂̑傫����Ԃ�
	int size(int x) {
		return sizes[find(x)];
	}
};
//int height, width;
//�֐���
/*  �摜name��ǂݍ����map[][]�ɓ�l�����ĕۑ�  */
void loadMap(char *name, int **map, int *height, int *width);
/*  �n�}�̕`��  */
void drawMap(int **map, int height, int width, tMouse d);
/*  ���x���ԍ����w�肵���n�}�̕`��  */
void drawMap_label(int **map, int height, int width, tMouse d, int label);
/*  ���_�ԍ����w�肵���n�}�̕`��  */
void drawMap_ver(int **map, int height, int width, tMouse d, tVer ver);
/*  �Ӕԍ����w�肵���n�}�̕`��  */
void drawMap_edge(int **map, int height, int width, tMouse d, tEdge edge);
/*  ��ʂ̕`��E�ۑ�  */
void saveScreen(int **map, int height, int width, tMouse d, char *filename);
/*  �}�E�X�̃C�x���g����  */
tMouse mouseEvent(tMouse d);
/*  �א���  */
void thinning(int **map, int height, int width);
int N(int *v);
int S(int *v);
/*  ���x�����O �߂�l�̓��x����  */
int labeling(int **map, int height, int width, int *label_area);
void search(int **map,int map_label[][MAPSIZE], int w, int h, int x, int y, int label, int *label_area);
/*  ���̈���Ȃ���  */
void connectSmallArea(int **map, int height, int width, int *label_area);
/*  ���̈�̍폜  */
void deleteSmallArea(int **map, int height, int width, int *label_area);
bool search2(int **map, int height, int width, int startx, int starty, int goalx, int goaly);
/*  ���H�쐻1(���x�����O���g�p)  */
void makeMaze1(int **map, int height, int width, int cut_count, int *label_area);
/*  ���H�쐻2(�T�����g�p)  */
void makeMaze2(int **map, int height, int width, int cut_count);
/*  ���H�쐻3(�O���t���g�p)  */
void makeMaze3(int **map, int height, int width, int vernum, std::vector<tEdge> &edge, tMouse d);
/*  ���_(�����_)�o�^ �߂�l�͒��_��  */
int entryVer(int **map, int height, int width, std::vector<tVer> &ver);
int countAround(int **map, int x, int y);
/*  �ӓo�^  */
int entryEdge(int **map, int height, int width, std::vector<tEdge> &edge, std::vector<tVer> &ver);
/*  �ӂ̍폜  */
bool deleteEdge(int **map, int height, int width, tEdge edge);
/*  tEdge��vector�z��̃V���b�t��  */
void shuffleEdge(std::vector<tEdge> &edge);
/*  tEdge.src.num�EtEdge.dst.num �� tVer.num�Ƃ̑Ή��t��  */
void associateEdgeNumber(std::vector<tVer> &ver, std::vector<tEdge> &edge);

int WINAPI WinMain( HINSTANCE hInstance, HINSTANCE hPrevInstance, LPSTR lpCmdLine, int nCmdShow )
{
	//�e��ݒ�
	//SetGraphMode(WINDOWX, WINDOWY, 32);
    ChangeWindowMode( TRUE );
	SetAlwaysRunFlag(TRUE);

    // �c�w���C�u�����̏�����
    if( DxLib_Init() < 0 ){
		return -1;
	}

	//�ϐ���
	static int map[MAPSIZE][MAPSIZE];//�n�}�̂ق����
	int *map_address[MAPSIZE];		 //���̖��̒ʂ�
	int height, width;				 //�n�}�̕�
	int save_height, save_width;	 //�ۑ��p�̒n�}�̕�(height��width��MAPSIZE�̂ǂ��炩�̏�������)
	tMouse d = {0, 0, 1};            //map�̒��g��\������ۂ̏��
	tMouse d_old = d;
	int dispmode=3;                  //�`�惂�[�h

	int labelnum;                    //�����x����
	int watch_label = 1;             //���������x���ԍ�
	int label_area[10000]={0};       //�e���x���̖ʐ�

	int vernum;                      //���_�̐�
	std::vector<tVer> ver;           //���_�o�^�p
	int watch_ver = 0;               //���������_�ԍ�

	int edgenum;                     //�ӂ̐�
	std::vector<tEdge> edge;         //�ӓo�^�p
	int watch_edge = 0;              //�������Ӕԍ�

	for(int i=0;i<MAPSIZE;i++){
		map_address[i] = map[i];
	}

	//****************  �O������������   ****************//

    //�}�b�v�ǂݍ���(��l��)
    loadMap(FILENAME, map_address, &height, &width);
	SetGraphMode(width, height, 32);
	//�ۑ��̈�̐ݒ�
	//800 * 800�ʂ����E���ۂ�
	if(height < MAPSIZE){
		save_height = height;
	}else{
		save_height = MAPSIZE;
	}
	if(width < MAPSIZE){
		save_width = width;
	}else{
		save_width = MAPSIZE;
	}
	//800 * 800�ʂ����E���ۂ��̂ŁE�E�E
	if(save_height > 800) save_height = 800;
	if(save_width > 800) save_width = 800;

	//�`�悵�ĕۑ�
	saveScreen(map_address, save_height, save_width, d, "��l��.bmp");

	//�א���
	thinning(map_address, height, width);

    //�`�悵�ĕۑ�
	saveScreen(map_address, save_height, save_width, d, "�א���.bmp");

	//���x�����O
	labelnum = labeling(map_address, height, width, label_area);

	//���̈���Ȃ�Yo!
	connectSmallArea(map_address, height, width, label_area);
	for(int i = 0; i < height && i < MAPSIZE; i ++ ){
		for(int j = 0; j < width && j < MAPSIZE; j ++ ){
			if(map[j][i] > ROAD){
				map[j][i] = ROAD;
			}
		}
	}
	
	//�`�悵�ĕۑ�
	saveScreen(map_address, save_height, save_width, d, "���̈�ڑ�.bmp");

	thinning(map_address, height, width);
	labelnum = labeling(map_address, height, width, label_area);
	
	//���̈���폜��Yo!
	deleteSmallArea(map_address, height, width, label_area);
	for(int i = 0; i < height && i < MAPSIZE; i ++ ){
		for(int j = 0; j < width && j < MAPSIZE; j ++ ){
			if(map[j][i] > ROAD){
				map[j][i] = ROAD;
			}
		}
	}
	
	//�`�悵�ĕۑ�
	saveScreen(map_address, save_height, save_width, d, "���̈�폜.bmp");

	labelnum = labeling(map_address, height, width, label_area);
	
	if(labelnum > 1){
		printfDx("Error:���x����2�ȏ゠���");
		ScreenFlip();
		WaitKey();
		exit(1);
	}

	//���_�̓o�^
	vernum = entryVer(map_address, height,width, ver);
	//�ӂ̓o�^
	edgenum = entryEdge(map_address, height, width, edge, ver);
	//edge�ɓo�^�����ӂ�src�Edst�̔ԍ���ver�ɓo�^���Ă���ԍ��ɂ��킹��
	associateEdgeNumber(ver, edge);
	shuffleEdge(edge);
	shuffleEdge(edge);
	mouseEvent(d);
	//****************  �O���������܂�   ****************//

	//****************  ���H�쐻��������  ****************//
	//makeMaze1(map_address, height, width, CUTCOUNT, label_area);
	//makeMaze2(map_address, height, width, CUTCOUNT);
	makeMaze3(map_address, height, width, vernum, edge, d);
	//****************  ���H�쐻�����܂�  ****************//

	//���˂Ƀ}�E�X�C�x���g�����s
	mouseEvent(d);
	//���_�̓o�^
	vernum = entryVer(map_address, height,width, ver);
	//�ӂ̓o�^
	edgenum = entryEdge(map_address, height, width, edge, ver);

	//**************************  ��������\��  **************************//
	bool isdraw = true;
	bool ishide = false;//�������B����
	while (ProcessMessage() == 0 && CheckHitKey(KEY_INPUT_ESCAPE) == 0){
		clsDx();
		d = mouseEvent(d);   //�}�E�X�̃C�x���g����
		if(d.x != d_old.x || d.y != d_old.y || d.size != d_old.size || isdraw){
			clsDx();
			ClearDrawScreen();
			switch(dispmode){
			case 0://�}�b�v�̕`��
				drawMap(map_address, height, width, d);
				break;
			case 1://���x�����w�肵���}�b�v�̕`��
				drawMap_label(map_address, height, width, d, watch_label);
				break;
			case 2://���_���w�肵���}�b�v�̕`��
				drawMap_ver(map_address, height, width, d, ver[watch_ver]);
				break;
			case 3://�Ӕԍ����w�肵���}�b�v�̕`��
				drawMap_edge(map_address, height, width, d, edge[watch_edge]);
				break;
			}
			if(!ishide){
				switch(dispmode){
				case 0://�}�b�v�̕`��
					printfDx("edgenum:%d\n",edgenum);
					break;
				case 1://���x�����w�肵���}�b�v�̕`��
					printfDx("label:%d  area:%d labelnum:%d\n",watch_label,label_area[watch_label],labelnum);
					break;
				case 2://���_���w�肵���}�b�v�̕`��
					printfDx("ver:%d x:%d y:%d  vernum:%d\n",ver[watch_ver].num, ver[watch_ver].x, ver[watch_ver].y,vernum);
					break;
				case 3://�Ӕԍ����w�肵���}�b�v�̕`��
					printfDx("edge:%d cost:%d   edgenum:%d\n", watch_edge, edge[watch_edge].cost, edgenum);
					printfDx("src_x:%d src_y:%d\n", edge[watch_edge].src.x, edge[watch_edge].src.y);
					printfDx("dst_x:%d dst_y:%d\n", edge[watch_edge].dst.x, edge[watch_edge].dst.y);
					break;
				}
				printfDx(" a : �`�惂�[�h�ύX\n");
				printfDx(" b : �ݒ�ύX\n");
				printfDx(" c : �������B��/�\������\n");
				printfDx(" d : �ӂ̍폜\n");
				printfDx(" e : ��ʂ̕ۑ�\n");
			}
			ScreenFlip();
			isdraw = false;
		}
		//�`�惂�[�h�̕ύX
		if(CheckHitKey( KEY_INPUT_A ) == 1){
			WaitTimer(100);
			clsDx();
			ClearDrawScreen();
			ScreenFlip();
			printfDx("�`�惂�[�h�̕ύX\n");
			printfDx(dispmode==0 ? "��" : "  ");
			printfDx(" 0 : �}�b�v�̕`��\n");
			printfDx(dispmode==1 ? "��" : "  ");
			printfDx(" 1 : ���x�����w�肵���}�b�v�̕`��\n");
			printfDx(dispmode==2 ? "��" : "  ");
			printfDx(" 2 : ���_���w�肵���}�b�v�̕`��\n");
			printfDx(dispmode==3 ? "��" : "  ");
			printfDx(" 3 : �ӂ��w�肵���}�b�v�̕`��\n");
			dispmode = KeyInputNumber( 0 , 96 , 3 , 0, FALSE);
			isdraw = true;
		}
		//�e�`��̐ݒ�ύX
		else if(CheckHitKey( KEY_INPUT_B ) == 1){
			WaitTimer(100);
			clsDx();
			ClearDrawScreen();
			ScreenFlip();
			switch(dispmode){
			case 0:
				break;
			case 1:
				printfDx("���x���ԍ������>");
				watch_label = KeyInputNumber( 0 , 32 , labelnum , 1, FALSE);
				break;
			case 2:
				printfDx("���_�ԍ������>");
				watch_ver = KeyInputNumber( 0 , 16 , vernum-1 , 0, FALSE);
				break;
			case 3:
				printfDx("�Ӕԍ������>");
				watch_edge = KeyInputNumber( 0 , 16 , edgenum-1 , 0, FALSE);
				break;
			}
			clsDx();
			ClearDrawScreen();
			isdraw = true;
		}
		//�������B���܂���?
		else if(CheckHitKey( KEY_INPUT_C ) == 1){
			WaitTimer(100);
			ishide = !ishide;
			isdraw = true;
		}
		//�ӂ̍폜
		else if(CheckHitKey( KEY_INPUT_D ) == 1){
			WaitTimer(100);
			clsDx();
			ClearDrawScreen();
			printfDx("�폜�������ӂ���͂��Ă�������\n");
			int delete_edge = KeyInputNumber( 0 , 32 , edgenum-1 , 0, FALSE);
			deleteEdge(map_address, height, width, edge[delete_edge]);
			isdraw =true;
		}
		//��ʂ̕ۑ�
		else if(CheckHitKey( KEY_INPUT_E ) == 1){
			WaitTimer(100);
			saveScreen(map_address, save_height, save_width, d, "���H.bmp");
			isdraw =true;
		}
		//���ڂ���ӂ�1���₷
		else if(CheckHitKey( KEY_INPUT_SPACE ) == 1){
			WaitTimer(100);
			watch_edge++;
			isdraw =true;
		}
		d_old = d;
	}
	//**************************  �����܂ŕ\��  **************************//

    // �c�w���C�u�����̌�n��
    DxLib_End();

    // �\�t�g�̏I��
    return 0;
}


//�֐���
//�摜name��ǂݍ����map[][]�ɓ�l�����ĕۑ�
void loadMap(char *name, int **map, int *height, int *width){
	int handle, r, g, b, a;
	handle = LoadSoftImage(name);
	GetSoftImageSize( handle, width, height );
	for(int i = 0; i < *height && i < MAPSIZE; i ++ ){
		for(int j = 0; j < *width && j < MAPSIZE; j ++ ){
			// �P�h�b�g�̐F���擾
			GetPixelSoftImage( handle, j, i, &r, &g, &b, &a ) ;
			//if(r >= 230 && g >= 230 && b >= 230){
			if(r >= 85 && g >= 85 && b >= 85){
				map[j][i] = ROAD;
			}else{
				map[j][i] = WALL;
			}
		}
	}
	DeleteSoftImage( handle );
}

//�א���
/*�Q�l:http://www.cellstat.net/thinning/
�א�����2�̏���X,Y��A���œK�p����
���ډ�f�Ƃ��̎���̒l v[9] ��ݒ�
		v[8] v[1] v[2]
		v[7] v[0] v[3]
		v[6] v[5] v[4]

����ɏ�������̂��߂̊֐�N(v)��S(v)���`
N(v):v[1]�`v[8]�̂���1(��)�ł�����̂̐�
S(v):v[1] > v[2] > v[3] > v[4] > �c > v[7] > v[8] > v[1] �ƈ�����邤���ŗׂ荇��2��0��1�ňقȂ���̂̐�

����X
�摜�̊e��f���������邽�߂�x,y�ɂ��Ă�2�d��for���[�v���񂵁A���X�̉�f�̒l��1(��)���������̂ɑ΂��āA
�ȉ��̏����𓯎��ɖ������ꍇ�ɂ͒l��0(��)�ɍX�V
����1: 2 <= N(v) <= 6
����2: S(v) == 1
����3: v[1] * v[3] * v[5] == 0
����4: v[3] * v[5] * v[7] == 0

����Y
�摜�̊e��f���������邽�߂�x,y�ɂ��Ă�2�d��for���[�v���񂵁A���X�̉�f�̒l��1(��)���������̂ɑ΂��āA
�ȉ��̏����𓯎��ɖ������ꍇ�ɂ͒l��0(��)�ɍX�V
����1: 2 <= N(v) <= 6
����2: S(v) == 1
����3: v[1] * v[3] * v[7] == 0
����4: v[1] * v[5] * v[7] == 0
*/
void thinning(int **map, int height, int width){
	bool complete = false; //�X�V���s��ꂽ��false�ɂȂ�true�ɂȂ�΍א����I��
	int count=0;
	while(!complete){
		complete = true;
		int v[9];

		for(int i = 1; i < height-1 && i < MAPSIZE-1; i ++ ){
			for(int j = 1; j < width-1 && j < MAPSIZE-1; j ++ ){
				//v[]�̐ݒ�
				v[8]=map[j-1][i-1]; v[1]=map[ j ][i-1]; v[2]=map[j+1][i-1];
				v[7]=map[j-1][ i ]; v[0]=map[ j ][ i ]; v[3]=map[j+1][ i ];
				v[6]=map[j-1][i+1]; v[5]=map[ j ][i+1]; v[4]=map[j+1][i+1];
				//����X
				if(map[j][i] == ROAD){
					if(2 <= N(v) && N(v) <= 6 && S(v) == 1 && v[1] * v[3] * v[5] == 0 && v[3] * v[5] * v[7] == 0){
						map[j][i] = WALL;
						complete = false;
					}
				}
				//����Y
				if(map[j][i] == ROAD){
					if(2 <= N(v) && N(v) <= 6 && S(v) == 1 && v[1] * v[3] * v[7] == 0 && v[1] * v[5] * v[7] == 0){
						map[j][i] = WALL;
						complete = false;
					}
				}
			}
		}
	}
}
int N(int *v){
	int count = 0;
	for(int i=1;i<9;i++){
		if(v[i] == ROAD){
			count++;
		}
	}
	return count;
}
int S(int *v){
	int count = 0;
	int old = v[8];
	for(int i=1;i<9;i++){
		if(old == 0 && v[i] == 1){
			count++;
		}
		old = v[i];
	}
	return count;
}

/*�ċA�L�O�i(����:�{��)

int getFinalDst(int i,int* dst){
	if(i == dst[i])
		return i;
	dst[i] = getFinalDst(dst[i], dst);
	return dst[i];
}


int labelClose(int* dst){
	int cnv[15000] = {0};
	int label_num = 0;
	for(int i=1;i<15000;i++){
		if(cnv[ dst[i] ] == 0){
			label_num++;
			cnv[ dst[i] ] = label_num;
		}
	}
	for(int i=1;i<15000;i++){
		dst[i] = cnv[ dst[i] ];
	}
	return label_num;
}


//���x�����O �߂�l�̓��x����
//�Q�l�Fhttp://imagingsolution.blog107.fc2.com/blog-entry-193.html

int labeling(int **map, int height, int width){
	static int map_label[MAPSIZE][MAPSIZE]={0}; //���x�����O�p�̃}�b�v
	int labelnum=0;	//���x���� �����Ԃ�
	int label_max=0;
	//int src[15000]; //���b�N�A�b�v�e�[�u��
	int dst[15000];

	for(int i=0;i<15000;i++){//���b�N�A�b�v�e�[�u���̏�����
		//src[i] = i;
		dst[i] = i;
	}

	//�����胉�x�����O���͂��߂܂�
	//���X�^�X�L����
	//��Ɉ�ԍ�������Ă���
	if(map[0][0] == ROAD){
		map_label[0][0] = ++labelnum;
	}

	for(int i = 0; i < height && i < MAPSIZE; i ++ ){
		for(int j = 0; j < width && j < MAPSIZE; j ++ ){
			if(i == 0 && j == 0) continue;//map[0][0]�͂Ƃ΂�

			if(j==0){//��ԏ�̍s�͏���Q�Ƃ��Ȃ�
				if(map[j][i] == ROAD){//���ډ�f�����Ȃ獶������
					if(map_label[j][i-1] != WALL){//�������Ȃ炻�̃��x���𒍖ډ�f�ɂ�����
						map_label[j][i] = map_label[j][i-1];
					}else{//�Ⴆ�΍Ō�ɂӂ������x���ԍ��̂ЂƂ傫�Ȕԍ����ӂ�
						map_label[j][i] = ++labelnum;
					}
				}
			}else if(i==0){//��ԍ��̍s�͍����Q�Ƃ��Ȃ�
				if(map[j][i] == ROAD){//���ډ�f�����Ȃ�������
					if(map[j-1][i] != WALL){//�オ���Ȃ炻�̃��x���𒍖ډ�f�ɂ�����
						map_label[j][i] = map_label[j-1][i];
					}else{//�Ⴆ�΍Ō�ɂӂ������x���ԍ��̂ЂƂ傫�Ȕԍ����ӂ�
						map_label[j][i] = ++labelnum;
					}
				}
			}else{//����ȊO�̂Ƃ���͍��Ə���Q�Ƃ���
				if(map[j][i] == ROAD){
					if(mini(map_label[j][i-1], map_label[j-1][i]) != -1){//�����Ƃ�0�łȂ����
						map_label[j][i] = mini(map_label[j][i-1], map_label[j-1][i]);//���ډ�f�̃��x���ԍ����������ق��̃��x���ԍ��ɂ��킹��
						int new_dst = mini(map_label[j][i-1], map_label[j-1][i]);
						while(new_dst != )
						dst[ dst[max(map_label[j][i-1], map_label[j-1][i])] ] = new_dst;//�O�ɓ����Ă���������������Ȃ��ƁE�E�E
						dst[max(map_label[j][i-1], map_label[j-1][i])] = new_dst;//���b�N�A�b�v�e�[�u���ɏ�������

					}else{//����0�Ȃ�ΐV���ȃ��x���ԍ�������
						map_label[j][i] = ++labelnum;
					}
				}
			}
		}
	}
	//�������߂�Ƃ�by�{��
	for(int i=1;i<15000;i++){
		getFinalDst(i, dst);
	}
	int label_all = labelClose(dst);
	//�����܂�by�{��
	clsDx();
	ClearDrawScreen();
	printfDx("labeling completed!!\nlabel:%d\n",labelnum);
	ScreenFlip();

	//���b�N�A�b�v�e�[�u�����g���đΉ�������
	for(int i = 0; i < height && i < MAPSIZE; i ++ ){
		for(int j = 0; j < width && j < MAPSIZE; j ++ ){
			if(map[j][i] != 0){
				map[j][i] = dst[ map_label[j][i] ];
				label_max = max(label_max, map[j][i]);
			}
		}
	}

	return label_max;
}
//0�ȊO�̏������ق���Ԃ� ����0�Ȃ�-1��Ԃ�
int mini(int a,int b){
	if(a == 0 && b == 0){
		return -1;
	}else{
		if(a == 0){
			return b;
		}else if(b == 0){
			return a;
		}else if(a < b){
			return a;
		}else{
			return b;
		}
	}
}
�����܂ōċA�L�O�i
*/



int labeling(int **map, int height, int width, int *label_area){
	static int map_label[MAPSIZE][MAPSIZE]={0}; //���x�����O�p�̃}�b�v
	int labelnum=0;	//���x���� �����Ԃ�

	for(int i = 0; i < height && i < MAPSIZE; i ++ ){
		for(int j = 0; j < width && j < MAPSIZE; j ++ ){
			map_label[j][i] = 0;
		}
	}
	for(int i = 0; i < height && i < MAPSIZE; i ++ ){
		for(int j = 0; j < width && j < MAPSIZE; j ++ ){
			if(map[j][i] == ROAD && map_label[j][i] == 0){
				search(map, map_label, width, height, j, i, ++labelnum, label_area);
			}
		}
	}
	for(int i = 0; i < height && i < MAPSIZE; i ++ ){
		for(int j = 0; j < width && j < MAPSIZE; j ++ ){
			map[j][i] = map_label[j][i];
		}
	}
	return labelnum;
}

void search(int **map,int map_label[][MAPSIZE], int w, int h, int x, int y, int label, int *label_area){
	//���x�����O�̒T�� ���������
	std::stack<std::pair<int, int>> p;
	p.push(std::make_pair(x, y));
	int count = 0;
	while(! p.empty()) {
		std::pair<int, int> pos = p.top();
		p.pop();
		int px = pos.first;
		int py = pos.second;
		map_label[px][py] = label;
		label_area[label]++;
		if (px - 1 >= 0 && map_label[px-1][py] == 0 && map[px-1][py] == ROAD) p.push(std::make_pair(px - 1, py));
		if (px + 1 < w && map_label[px+1][py] == 0 && map[px+1][py] == ROAD) p.push(std::make_pair(px + 1, py));
		if (py - 1 >= 0 && map_label[px][py-1] == 0 && map[px][py-1] == ROAD) p.push(std::make_pair(px, py - 1));
		if (py + 1 < h && map_label[px][py+1] == 0 && map[px][py+1] == ROAD) p.push(std::make_pair(px, py + 1));
	}
}
/*  ���̈���Ȃ���  
���x���t�����ꂽ�̈�̖ʐς�SMALL�ȉ��̏��̈�ɑ΂���
�΂ߕ����̘A�������āA����������f(��)�����g�ƈႤ���x���ԍ��Ȃ�
�������Ȃ���
               *                 *                 *
2 0 0   2 0 0  *  0 0 2   0 0 2  *  0 0 0   0 0 0  *  0 0 0   0 0 0
0 1 0 > 1 1 0  *  0 1 0 > 0 1 1  *  0 1 0 > 1 1 0  *  0 1 0 > 0 1 1
0 0 0   0 0 0  *  0 0 0   0 0 0  *  2 0 0   2 0 0  *  0 0 2   0 0 2
               *                 *                 *
*/
void connectSmallArea(int **map, int height, int width, int *label_area){
	for(int i = 0; i < height && i < MAPSIZE; i ++ ){
		for(int j = 0; j < width && j < MAPSIZE; j ++ ){
			if(map[j][i] != 0 && label_area[ map[j][i] ] <= SMALL){
				if(i != 0 && j != 0){
					if(map[j-1][i-1] >= 1 && map[j][i] != map[j-1][i-1]) map[j-1][i] = ROAD;
				}
				if(i != 0){
					if(map[j+1][i-1] >= 1 && map[j][i] != map[j+1][i-1]) map[j+1][i] = ROAD;
				}
				if(j != 0){
					if(map[j-1][i+1] >= 1 && map[j][i] != map[j-1][i+1]) map[j-1][i] = ROAD;
				}
				if(map[j+1][i+1] >= 1 && map[j][i] != map[j+1][i+1]) map[j+1][i] = ROAD;
			}
		}
	}
}

/*  ���̈�̍폜  */
void deleteSmallArea(int **map, int height, int width, int *label_area){
	for(int i = 0; i < height && i < MAPSIZE; i ++ ){
		for(int j = 0; j < width && j < MAPSIZE; j ++ ){
			if(label_area[ map[j][i] ] <= SMALL){
				map[j][i] = 0;
			}
		}	
	}
}

/*  ���H�쐻1(���x�����O���g�p)  */
void makeMaze1(int **map, int height, int width, int cut_count, int *label_area){
	//XY a[10];
	int count = 0;
	while(cut_count != 0){
		//�����_���ɑI�񂾓_������f�ɕς���
		/*
		for(int i=0;i<10;i++){
			int x = GetRand(min(width,MAPSIZE));
			int y = GetRand(min(height,MAPSIZE));
			if(map[x][y] == WALL){
				i--;
				continue;
			}
			map[x][y] = WALL;
			a[i].x = x; a[i].y = y;
		}
		*/
		///*
		for(int i = 0; i < height && i < MAPSIZE; i ++ ){
			for(int j = 0; j < width && j < MAPSIZE; j ++ ){
				if(map[j][i] > ROAD){
					map[j][i] = ROAD;
				}
			}
		}
		int x = GetRand(min(width,MAPSIZE))+1;
		int y = GetRand(min(height,MAPSIZE))+1;
		if(map[x][y] == WALL){
			continue;
		}
		map[x][y] = WALL;
		//*/
		//���x�����O���g�����A�����m�F�@
		//ClearDrawScreen();
		//printfDx("making�c \n");
		//ScreenFlip();
		if(labeling(map, height, width, label_area) == 1){
			cut_count--;
			//clsDx();
			ClearDrawScreen();
			//printfDx("making�c cut_count:%d  x:%d  y:%d\n",cut_count,x,y);
			printfDx("making�c cut_count:%d\n",cut_count);
			ScreenFlip();
		}else{
			//ClearDrawScreen();
			//printfDx("making�c false\n");
			//ScreenFlip();
			//for(int i=0;i<10;i++){
			//	map[a[i].x][a[i].y] = ROAD;
			//}
			map[x][y]=ROAD;
		}
		//cut_count--;
		ClearDrawScreen();
		//printfDx("making�c cut_count:%d\n",cut_count);
		ScreenFlip();
	}
}

/*  ���H�쐻2(�T�����g�p)  */
void makeMaze2(int **map, int height, int width, int cut_count){
	while(cut_count != 0){
		//�����_���ɑI�񂾓_������f�ɕς���
		
		int x = GetRand(min(width,MAPSIZE))+1;
		int y = GetRand(min(height,MAPSIZE))+1;
		if(map[x][y] == WALL)continue;
		map[x][y] = WALL;
		//**********    �T�����g�����A�����m�F�@    **********//
		//  ��
		//������
		//  ��
		if(map[x-1][y] == ROAD && map[x][y-1] == ROAD){
			if(search2(map,height, width, x-1, y, x, y-1)){}
			else{
				map[x][y]=ROAD;
				continue;
			}
		}
		//  ��
		//������
		//  ��
		if(map[x-1][y] == ROAD && map[x+1][y] == ROAD){
			if(search2(map,height, width, x-1, y, x+1, y)){}
			else{
				map[x][y]=ROAD;
				continue;
			}
		}
		//  ��
		//������
		//  ��
		if(map[x-1][y] == ROAD && map[x][y+1] == ROAD){
			if(search2(map,height, width, x-1, y, x, y+1)){}
			else{
				map[x][y]=ROAD;
				continue;
			}
		}
		//  ��
		//������
		//  ��
		if(map[x][y-1] == ROAD && map[x+1][y] == ROAD){
			if(search2(map,height, width, x, y-1, x+1, y)){}
			else{
				map[x][y]=ROAD;
				continue;
			}
		}
		//  ��
		//������
		//  ��
		if(map[x][y-1] == ROAD && map[x][y+1] == ROAD){
			if(search2(map,height, width, x, y-1, x, y+1)){}
			else{
				map[x][y]=ROAD;
				continue;
			}
		}
		//  ��
		//������
		//  ��
		if(map[x+1][y] == ROAD && map[x][y+1] == ROAD){
			if(search2(map,height, width, x+1, y, x, y+1)){}
			else{
				map[x][y]=ROAD;
				continue;
			}
		}
		if(CheckHitKey( KEY_INPUT_S ) == 1){
			break;
		}
		//**********    �T�����g�����A�����m�F�@�����܂�    **********//
		cut_count--;
		ClearDrawScreen();
		printfDx("making�c cut_count:%d\n",cut_count);
		ScreenFlip();
	}
}

/*  ���H�쐻3(�O���t���g�p)  */
void makeMaze3(int **map, int height, int width, int vernum, std::vector<tEdge> &edge, tMouse d){
	UnionFind uf(vernum);
	clsDx();
	ClearDrawScreen();
	printfDx("Map making start...\n");
	ScreenFlip();
	int count=0;
	for(int i=0;i<edge.size();i++){
		//src��dst�������O���[�v�ɑ����Ă��Ȃ���΂��̕ӂ͍폜������src��dst�𓯂��O���[�v�ɂ���
		if(!uf.same(edge[i].src.num, edge[i].dst.num)){
			uf.unite(edge[i].src.num, edge[i].dst.num);
		}
		//src��dst�������O���[�v�ɑ����Ă����炻�̕ӂ��폜
		else{
			count++;
			deleteEdge(map, height, width, edge[i]);
		}
	}
	clsDx();
	ClearDrawScreen();
	printfDx("Map making end!!!!\ncount : %d\n",count);
	ScreenFlip();
	WaitTimer(1000);
}

/*  �A�����Ă��邩�̊m�F  */
bool search2(int **map, int height, int width, int startx, int starty, int goalx, int goaly){
	std::vector<std::vector<char>> map_flag;
	map_flag.resize(MAPSIZE);
	for( int i=0; i<MAPSIZE; i++ ){
		map_flag[i].resize(MAPSIZE);
	}
	std::stack<std::pair<int, int>> p;
	p.push(std::make_pair(startx, starty));
	while(! p.empty()) {
		std::pair<int, int> pos = p.top();
		p.pop();
		int px = pos.first;
		int py = pos.second;
		if(px == goalx && py == goaly){
			return true;
		}
		map_flag[px][py] = 1;
		if (px - 1 >= 0     && map_flag[px-1][py] == 0 && map[px-1][py] == ROAD) p.push(std::make_pair(px - 1, py));
		if (px + 1 < width  && map_flag[px+1][py] == 0 && map[px+1][py] == ROAD) p.push(std::make_pair(px + 1, py));
		if (py - 1 >= 0     && map_flag[px][py-1] == 0 && map[px][py-1] == ROAD) p.push(std::make_pair(px, py - 1));
		if (py + 1 < height && map_flag[px][py+1] == 0 && map[px][py+1] == ROAD) p.push(std::make_pair(px, py + 1));
	}
	return false;
}

/*  �n�}�̕`��  */
void drawMap(int **map, int height, int width, tMouse d){
	static unsigned int white = GetColor(255, 255, 255);
	static unsigned int red = GetColor(255, 0, 0);
	static unsigned int blue = GetColor(0, 0, 255);
	int x, y;

	for(int i = 0; i < height && i < MAPSIZE; i ++ ){
		for(int j = 0; j < width && j < MAPSIZE; j ++ ){
			x = j * d.size + d.x;
			y = i * d.size + d.y;
			if(map[j][i] == ROAD){
				DrawBox(x, y, x + d.size, y + d.size, white, TRUE);
			}else if(map[j][i] == WALL){}
			else if(map[j][i] % 2 == 0){
				DrawBox(x, y, x + d.size, y + d.size, red, TRUE);
			}
			else if(map[j][i] % 2 == 1){
				DrawBox(x, y, x + d.size, y + d.size, blue, TRUE);
			}
		}
	}
}

/*  ���x���ԍ����w�肵���n�}�̕`��  */
void drawMap_label(int **map, int height, int width, tMouse d, int label){
	static unsigned int white = GetColor(255, 255, 255);
	static unsigned int red = GetColor(255, 0, 0);
	static unsigned int blue = GetColor(0, 0, 255);
	int x, y;

	for(int i = 0; i < height && i < MAPSIZE; i ++ ){
		for(int j = 0; j < width && j < MAPSIZE; j ++ ){
			x = j * d.size + d.x;
			y = i * d.size + d.y;
			if(map[j][i] == WALL){}
			else if(map[j][i] == label){
				DrawBox(x, y, x + d.size, y + d.size, red, TRUE);
			}
			else{
				DrawBox(x, y, x + d.size, y + d.size, white, TRUE);
			}
		}
	}
	for(int i = 1; i < height && i < MAPSIZE; i ++ ){
		for(int j = 1; j < width && j < MAPSIZE; j ++ ){
			if(map[j][i] == ROAD && countAround(map, j, i) >= 3){
				x = j * d.size + d.x;
				y = i * d.size + d.y;
				DrawBox(x, y, x + d.size, y + d.size, white, TRUE);
			}
		}
	}
}

/*  ���_�ԍ����w�肵���n�}�̕`��  */
void drawMap_ver(int **map, int height, int width, tMouse d, tVer ver){
	static unsigned int white = GetColor(255, 255, 255);
	static unsigned int red = GetColor(255, 0, 0);
	static unsigned int blue = GetColor(0, 0, 255);
	int x, y;

	for(int i = 0; i < height && i < MAPSIZE; i ++ ){
		for(int j = 0; j < width && j < MAPSIZE; j ++ ){
			x = j * d.size + d.x;
			y = i * d.size + d.y;
			if(map[j][i] == WALL){}
			else if(map[j][i] == ROAD){
				DrawBox(x, y, x + d.size, y + d.size, white, TRUE);
			}
		}
	}
	x = ver.x * d.size + d.x;
	y = ver.y * d.size + d.y;
	DrawBox(x, y, x + d.size, y + d.size, red, TRUE);
}

/*  �Ӕԍ����w�肵���n�}�̕`��  */
void drawMap_edge(int **map, int height, int width, tMouse d, tEdge edge){
	static unsigned int white = GetColor(255, 255, 255);
	static unsigned int red   = GetColor(255,   0,   0);
	static unsigned int green = GetColor(  0, 255,   0);
	static unsigned int blue  = GetColor(  0,   0, 255);
	int x,y;
	int srcx, srcy, dstx, dsty;
	//         �� �� �� �E
	int dx[4]={-1, 0, 1, 0};
	int dy[4]={ 0,-1, 0, 1};
	//�K�ꂽ���ǂ����̊m�F�p
	std::vector<std::vector<char>> map_flag;
	map_flag.resize(MAPSIZE);
	for( int i=0; i<MAPSIZE; i++ ){
		map_flag[i].resize(MAPSIZE);
	}

	for(int i = 0; i < height && i < MAPSIZE; i ++ ){
		for(int j = 0; j < width && j < MAPSIZE; j ++ ){
			x = j * d.size + d.x;
			y = i * d.size + d.y;
			if(map[j][i] == WALL){}
			else if(map[j][i] == ROAD){
				DrawBox(x, y, x + d.size, y + d.size, white, TRUE);
			}
		}
	}

	//src�`��(��)
	srcx = edge.src.x * d.size + d.x;
	srcy = edge.src.y * d.size + d.y;
	DrawBox(srcx, srcy, srcx + d.size, srcy + d.size, red, TRUE);
	//dst�`��(��)
	dstx = edge.dst.x * d.size + d.x;
	dsty = edge.dst.y * d.size + d.y;
	DrawBox(dstx, dsty, dstx + d.size, dsty + d.size, blue, TRUE);

	//�ӂ̕`��
	std::stack<std::pair<int, int>> p;
	//�ӂ�src��angle�����ɕ`�悵�Ȃ���dst��T��
	p.push(std::make_pair(edge.src.x+dx[edge.angle], edge.src.y+dy[edge.angle]));
	map_flag[edge.src.x][edge.src.y] = 1;
	while(! p.empty()){
		std::pair<int, int> pos = p.top();
		p.pop();
		int px = pos.first;
		int py = pos.second;
		map_flag[px][py] = 1;
		//���_dst��K���܂Ń��[�v
		if(px == edge.dst.x && py == edge.dst.y){
			break;
		}else{
			x = px * d.size + d.x;
			y = py * d.size + d.y;
			DrawBox(x, y, x + d.size, y + d.size, green, TRUE);
			if (px - 1 >= 0     && map[px-1][py] != WALL && map_flag[px-1][py] == 0) p.push(std::make_pair(px - 1, py));
			if (px + 1 < width  && map[px+1][py] != WALL && map_flag[px+1][py] == 0) p.push(std::make_pair(px + 1, py));
			if (py - 1 >= 0     && map[px][py-1] != WALL && map_flag[px][py-1] == 0) p.push(std::make_pair(px, py - 1));
			if (py + 1 < height && map[px][py+1] != WALL && map_flag[px][py+1] == 0) p.push(std::make_pair(px, py + 1));
		}
	}
}

/*  ��ʂ̕`��E�ۑ�  */
void saveScreen(int **map, int height, int width, tMouse d, char *filename){
	ClearDrawScreen();
	drawMap(map, height, width, d);
	ScreenFlip();
	SaveDrawScreen( 0, 0, width, height, filename);
}

/*  ���_(�����_)�o�^ �߂�l�͒��_��  */
int entryVer(int **map, int height, int width, std::vector<tVer> &ver){
	int ver_count = 0;
	for(int i = 1; i < height && i < MAPSIZE; i ++ ){
		for(int j = 1; j < width && j < MAPSIZE; j ++ ){
			if(map[j][i] == ROAD && countAround(map, j, i) >= 3){
				ver.push_back(tVer(ver_count++, j, i));
			}
		}
	}
	return ver_count;
}
int countAround(int **map, int x, int y){
	int count = 0;
	if(map[x-1][ y ] == ROAD) count++;if(map[x+1][ y ] == ROAD) count++;
	if(map[ x ][y-1] == ROAD) count++;
	if(map[ x ][y+1] == ROAD) count++;
	return count;
}

/*  �ӂ̓o�^  */
int entryEdge(int **map, int height, int width, std::vector<tEdge> &edge, std::vector<tVer> &ver){
	int edgecount = 0;
	//         �� �� �� �E
	int dx[4]={-1, 0, 1, 0};
	int dy[4]={ 0,-1, 0, 1};
	std::vector<std::vector<int>> mapcopy;
	mapcopy.resize(MAPSIZE);
	for( int i=0; i<MAPSIZE; i++ ){
		mapcopy[i].resize(MAPSIZE);
	}
	//map�̃R�s�[���Ƃ�
	for(int i = 1; i < height && i < MAPSIZE; i ++ ){
		for(int j = 1; j < width && j < MAPSIZE; j ++ ){
			mapcopy[j][i] = map[j][i];
		}
	}
	//���_��VERTEX(2)�ɕϊ�
	for(int i=0;i<ver.size();i++){
		mapcopy[ver[i].x][ver[i].y] = VERTEX;
	}
	//���_����T��
	for(int i=0;i<ver.size();i++){
		//�l�ߖT�T��
		for(int j=0;j<4;j++){//j=0(��),1(��),2(��),3(�E)
			int cost=0;
			std::stack<std::pair<int, int>> p;
			if(ver[i].x+dx[j] < 0 || ver[i].y+dy[j] < 0 || ver[i].x+dx[j] >= width || ver[i].y+dy[j] >= height){
				continue;
			}
			if(mapcopy[ver[i].x+dx[j]][ver[i].y+dy[j]] == WALL){
				continue;
			}
			p.push(std::make_pair(ver[i].x+dx[j], ver[i].y+dy[j]));
			while(! p.empty()){
				std::pair<int, int> pos = p.top();
				p.pop();
				int px = pos.first;
				int py = pos.second;
				//�����K��n�_�����_�Ȃ�T���J�n���_�ƍ����钸�_�ŕӂ�o�^
				if(mapcopy[px][py] == VERTEX && (px != ver[i].x || py != ver[i].y)){
					tVer src(ver[i].x, ver[i].y);
					tVer dst(px, py);
					edge.push_back(tEdge(src, dst, cost, j));
					edgecount++;
					break;
				}
				cost++;
				mapcopy[px][py] = WALL;
				if (px - 1 >= 0     && mapcopy[px-1][py] != WALL && (px-1 != ver[i].x || py != ver[i].y)){
					p.push(std::make_pair(px - 1, py));
				}
				if (px + 1 < width  && mapcopy[px+1][py] != WALL && (px+1 != ver[i].x || py != ver[i].y)){
					p.push(std::make_pair(px + 1, py));
				}
				if (py - 1 >= 0     && mapcopy[px][py-1] != WALL && (px != ver[i].x || py-1 != ver[i].y)){
					p.push(std::make_pair(px, py - 1));
				}
				if (py + 1 < height && mapcopy[px][py+1] != WALL && (px != ver[i].x || py+1 != ver[i].y)){
					p.push(std::make_pair(px, py + 1));
				}
			}
		}
	}
	return edgecount;
}

/*  �ӂ̍폜  */
bool deleteEdge(int **map, int height, int width, tEdge edge){
	int dx[4]={-1, 0, 1, 0};
	int dy[4]={ 0,-1, 0, 1};
	//�K�ꂽ���ǂ����̊m�F�p
	std::vector<std::vector<char>> map_flag;
	map_flag.resize(MAPSIZE);
	for( int i=0; i<MAPSIZE; i++ ){
		map_flag[i].resize(MAPSIZE);
	}
	//�ӂ̒T��
	std::stack<std::pair<int, int>> p;
	//�ӂ�src��angle�����ɍ폜���Ȃ���dst��T��
	p.push(std::make_pair(edge.src.x+dx[edge.angle], edge.src.y+dy[edge.angle]));
	map_flag[edge.src.x][edge.src.y] = 1;
	while(! p.empty()){
		std::pair<int, int> pos = p.top();
		p.pop();
		int px = pos.first;
		int py = pos.second;
		//���_dst��K���܂Ń��[�v
		if(px == edge.dst.x && py == edge.dst.y){
			return true;
		}else{
			map_flag[px][py] = 1;
			map[px][py] = WALL;
			if (px - 1 >= 0     && map[px-1][py] != WALL && map_flag[px-1][py] == 0) p.push(std::make_pair(px - 1, py));
			if (px + 1 < width  && map[px+1][py] != WALL && map_flag[px+1][py] == 0) p.push(std::make_pair(px + 1, py));
			if (py - 1 >= 0     && map[px][py-1] != WALL && map_flag[px][py-1] == 0) p.push(std::make_pair(px, py - 1));
			if (py + 1 < height && map[px][py+1] != WALL && map_flag[px][py+1] == 0) p.push(std::make_pair(px, py + 1));
		}
	}
	return false;
}

/*  tEdge��vector�z��̃V���b�t��  */
void shuffleEdge(std::vector<tEdge> &edge){
	//�R�X�g��0�̕ӂ͖������Ő擪����l�߂�
	int count=0;//�R�X�g��0�̕ӂ̐����J�E���g
	
	for(int i=0;i<edge.size();i++){
		int j = GetRand(edge.size()-1);
		tEdge t = edge[i];
		edge[i] = edge[j];
		edge[j] = t;
	}
	for(int i=0;i<edge.size();i++){
		if(edge[i].cost == 0){
			tEdge t     = edge[count];
			edge[count] = edge[i];
			edge[i]     = t;
			count++;
		}
	}
}

/*  tEdge.src.num�EtEdge.dst.num �� tVer.num�Ƃ̑Ή��t��  */
void associateEdgeNumber(std::vector<tVer> &ver, std::vector<tEdge> &edge){
	for(int i=0;i<edge.size();i++){
		for(int j=0;j<ver.size();j++){
			//edge��src�̒��_��ver�Ɠ����ꏊ�Ȃ�edge��src�̔ԍ���ver�̔ԍ��ɂ��킹��
			if(edge[i].src.x == ver[j].x && edge[i].src.y == ver[j].y){
				edge[i].src.num = ver[j].num;
			}
			//edge��dst�̒��_��ver�Ɠ����ꏊ�Ȃ�edge��dst�̔ԍ���ver�̔ԍ��ɂ��킹��
			else if(edge[i].dst.x == ver[j].x && edge[i].dst.y == ver[j].y){
				edge[i].dst.num = ver[j].num;
			}
		}
	}
}

//�}�E�X�̃C�x���g����
tMouse mouseEvent(tMouse d){
    int MouseInput;
    int MouseWheelRotVol;
    int x,y;
	int mouse_x, mouse_y;
	static int mouse_old_x=99999, mouse_old_y;

	if(mouse_old_x == 99999){//�ŏ��������s�������Ƃ���
		GetMousePoint(&mouse_old_x, &mouse_old_y);
		return d;
	}

	GetMousePoint(&mouse_x, &mouse_y);

    MouseInput = GetMouseInput();
    if(( MouseInput & MOUSE_INPUT_LEFT ) != 0 ){
        d.x += mouse_x - mouse_old_x;
        d.y += mouse_y - mouse_old_y;
    }
	//�u���b�N�T�C�Y��傫��(������)�����Ƃ��̕`��ʒu�̌v�Z
	x = (-d.x + mouse_x) / d.size;
	y = (-d.y + mouse_y) / d.size;
    MouseWheelRotVol = GetMouseWheelRotVol();
    if(MouseWheelRotVol > 0){
		d.size++;
        d.x -= x;
        d.y -= y;
    }
	else if(MouseWheelRotVol < 0 && d.size > 1){
		d.size--;
        d.x += x;
        d.y += y;
    }

	//mouse_old�̍X�V
	mouse_old_x = mouse_x;
	mouse_old_y = mouse_y;

	return d;
}