#include "vision.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <limits.h>
#include <ctype.h>

/* 帧校验：@ 与 * 之间所有 ASCII 字节逐字节 XOR */
static unsigned char xor_bytes(const char *s, size_t n) {
    unsigned char v=0;
    size_t i;
    for (i=0; i<n; ++i) v ^= (unsigned char)s[i];
    return v;
}

static void stop_vehicle(VisionContext *v) {
    v->pos.valid=0;
    if (v->stop) v->stop(v->user);
}

int Vision_SendBody(VisionContext *v, const char *body) {
    char head[VISION_FRAME_MAX], frame[VISION_FRAME_MAX];
    size_t n;
    int x,y;
    if (!v || !v->tx || !body) return 0;
    n=strlen(body);
    if (!n || n>VISION_BODY_MAX) return 0;
    for (size_t i=0; i<n; ++i)
        if ((unsigned char)body[i]<32 || (unsigned char)body[i]>126 ||
            body[i]=='*' || body[i]=='@') return 0;
    x=snprintf(head,sizeof(head),"%u,%u,%s",(unsigned)v->tx_seq,(unsigned)n,body);
    if (x<=0 || x>=(int)sizeof(head)) return 0;
    y=snprintf(frame,sizeof(frame),"@%s*%02X\n",head,
               (unsigned)xor_bytes(head,(size_t)x));
    if (y<=0 || y>=(int)sizeof(frame)) return 0;
    ++v->tx_seq;
    v->tx((const uint8_t *)frame,(size_t)y,v->user);
    return 1;
}

static int integer(const char *s, long *number) {
    char *end;
    long result;
    if (!s || !*s || !number) return 0;
    result=strtol(s,&end,10);
    if (end==s || *end) return 0;
    *number=result;
    return 1;
}

static int unsigned_value(const char *s, unsigned long *number) {
    char *end;
    unsigned long result;
    if (!s || !*s || !number || !isdigit((unsigned char)*s)) return 0;
    result=strtoul(s,&end,10);
    if (end==s || *end) return 0;
    *number=result;
    return 1;
}

/* 只接受校验和与长度均正确的完整帧；将原缓冲区原地拆开 */
static char *decode(char *line) {
    char *star,*first,*second,*body;
    unsigned long seq, length;
    unsigned chk;
    if (*line!='@') return NULL;
    star=strchr(line,'*');
    if (!star || strlen(star+1)!=2 || !isxdigit((unsigned char)star[1]) ||
        !isxdigit((unsigned char)star[2]) ||
        sscanf(star+1,"%2x",&chk)!=1) return NULL;
    if (xor_bytes(line+1,(size_t)(star-line-1)) != (unsigned char)chk) return NULL;
    *star=0;
    first=strchr(line+1,',');
    if (!first) return NULL;
    *first=0;
    if (!unsigned_value(line+1,&seq) || seq>65535u) return NULL;
    second=strchr(first+1,',');
    if (!second) return NULL;
    *second=0;
    if (!unsigned_value(first+1,&length) || length>VISION_BODY_MAX) return NULL;
    body=second+1;
    if (!length || strlen(body)!=length) return NULL;
    return body;
}

static int tokens(char *s, char **f, int cap) {
    int n=0;
    char *comma;
    while (n<cap && s && *s) {
        f[n++]=s;
        comma=strchr(s,',');
        if (!comma) return n;
        *comma=0;
        s=comma+1;
    }
    return 0;
}

static void set_text(char *dest, size_t cap, const char *s) {
    if (!cap) return;
    if (!s) s="";
    snprintf(dest,cap,"%s",s);
}

static int job_code_valid(unsigned code) {
    return (code==1 || (code>=11 && code<=16) ||
            (code>=21 && code<=23) || code==31 || code==32 ||
            code==40 || code==41 || code==42);
}

static int job_matches(unsigned code, const char *kind, long target) {
    if (code==41) return !strcmp(kind,"MATERIAL") && target==0;
    if (code==42) return !strcmp(kind,"MATERIAL") && target>=1 && target<=6;
    if (code>=11 && code<=16) return !strcmp(kind,"MATERIAL") && target==(long)(code-10);
    if (code>=21 && code<=23) return !strcmp(kind,"RING") && target==(long)(code-20);
    if (code==31) return !strcmp(kind,"EDGE") && target==0;
    if (code==32) return !strcmp(kind,"BLUE") && target==0;
    return 0;
}

static int code_is_valid(const char *s) {
    size_t i;
    if (!s || strlen(s)!=15) return 0;
    for (i=0; i<15; ++i) {
        if (i==3 || i==7 || i==11) {
            if (s[i]!='+') return 0;
        } else if (s[i]<'1' || s[i]>((i>=4 && i<=6) || (i>=12 && i<=14) ? '3' : '6')) return 0;
    }
    for (i=0; i<4; ++i) {
        size_t base=i*4;
        if (s[base]==s[base+1] || s[base]==s[base+2] ||
            s[base+1]==s[base+2]) return 0;
    }
    return 1;
}

static void handle(VisionContext *v, char *body, uint32_t now) {
    char *f[12];
    int n=tokens(body,f,12);
    long id,dx,dy,cx,cy,code,batch,index;
    if (!n) return;
    if (!strcmp(f[0],"READY") && n==2 && !strcmp(f[1],"IDLE")) {
        v->ready=1; v->phase=VISION_IDLE;
        stop_vehicle(v);
    } else if (!strcmp(f[0],"STATUS") && n==6 && !v->ready) {
        /* 若 READY,IDLE 启动帧丢失，可通过状态查询重新建立连接。 */
        if (!strcmp(f[1],"IDLE")) {
            v->ready=1;
            v->phase=VISION_IDLE;
            stop_vehicle(v);
        } else {
            Vision_SendBody(v,"CANCEL");
        }
    } else if (!strcmp(f[0],"ACK") && n==3 && !strcmp(f[1],"JOB")) {
        if (integer(f[2],&code) && v->phase==VISION_WAIT_ACK &&
            code==v->job_code) v->phase=VISION_RUNNING;
    } else if (!strcmp(f[0],"POS") && n==7) {
        if (v->phase!=VISION_RUNNING || !integer(f[2],&id) ||
            !integer(f[3],&dx) || !integer(f[4],&dy) ||
            !integer(f[5],&cx) || !integer(f[6],&cy) ||
            !job_matches(v->job_code,f[1],id) ||
            dx < -320 || dx > 320 || dy < -240 || dy > 240 ||
            cx < 0 || cx>=320 || cy<0 || cy>=240) return;
        v->pos.dx=(int16_t)dx; v->pos.dy=(int16_t)dy;
        v->pos.cx=(uint16_t)cx; v->pos.cy=(uint16_t)cy;
        v->pos.target=(uint8_t)id;
        set_text(v->pos.kind,sizeof(v->pos.kind),f[1]);
        v->pos.time_ms=now; v->pos.valid=1;
        v->wait_reason[0]=0;
    } else if (!strcmp(f[0],"ARRIVED") && n==5) {
        if (v->phase!=VISION_RUNNING || !integer(f[2],&id) ||
            !integer(f[3],&dx) || !integer(f[4],&dy) ||
            !job_matches(v->job_code,f[1],id)) return;
        v->result=VISION_RESULT_ARRIVED;
        v->phase=VISION_WAIT_IDLE;
        stop_vehicle(v);
    } else if (!strcmp(f[0],"PICK_READY") && n==4) {
        if (v->job_code!=42 || v->phase!=VISION_WAIT_IDLE ||
            !integer(f[1],&id) || !integer(f[2],&batch) ||
            !integer(f[3],&index) || id<1 || id>6 ||
            batch<1 || batch>2 || index<1 || index>3) return;
        v->pick_ready=1; v->pick_color=(uint8_t)id;
        v->pick_batch=(uint8_t)batch; v->pick_index=(uint8_t)index;
        v->result=VISION_RESULT_PICK_READY;
        stop_vehicle(v);
    } else if (!strcmp(f[0],"AREA_OK") && n==4) {
        if (v->phase==VISION_RUNNING && v->job_code==40)
            v->area_confirmed=1;
    } else if (!strcmp(f[0],"QR") && n==2) {
        if (code_is_valid(f[1])) {
            set_text(v->qr_code,sizeof(v->qr_code),f[1]);
            v->has_qr=1;
        }
    } else if (!strcmp(f[0],"DONE") && n==2) {
        if (!strcmp(f[1],"QR") && v->job_code==1)
            v->result=VISION_RESULT_QR_DONE;
        else if (!strcmp(f[1],"AREA") && v->job_code==40)
            v->result=VISION_RESULT_AREA_DONE;
    } else if (!strcmp(f[0],"WAIT") && n>=2) {
        set_text(v->wait_reason,sizeof(v->wait_reason),f[1]);
        stop_vehicle(v);
    } else if (!strcmp(f[0],"LOST") && n==3) {
        set_text(v->wait_reason,sizeof(v->wait_reason),"LOST");
        stop_vehicle(v);
    } else if (!strcmp(f[0],"TIMEOUT") && n==3) {
        v->result=VISION_RESULT_TIMEOUT;
        v->phase=VISION_WAIT_IDLE;
        stop_vehicle(v);
    } else if (!strcmp(f[0],"ERR") && n>=2) {
        set_text(v->last_error,sizeof(v->last_error),f[1]);
        v->result=VISION_RESULT_ERROR;
        v->phase=VISION_IDLE;   /* OpenMV 拒绝 JOB 时不会额外返回 IDLE */
        stop_vehicle(v);
    } else if (!strcmp(f[0],"CANCELLED")) {
        v->result=VISION_RESULT_CANCELLED;
        v->phase=VISION_WAIT_IDLE;
        v->pick_ready=0;
        stop_vehicle(v);
    } else if (!strcmp(f[0],"ACK") && n>=2 &&
               (!strcmp(f[1],"PICK_OK") || !strcmp(f[1],"PICK_FAIL"))) {
        v->pick_ready=0;
        v->pick_ack_pending=0;
    } else if (!strcmp(f[0],"ACK") && n>=2 &&
               (!strcmp(f[1],"RESET") || !strcmp(f[1],"AREA_RESET"))) {
        v->area_confirmed=0;
        v->pick_ready=0;
        if (!strcmp(f[1],"RESET")) v->has_qr=0;
    } else if (!strcmp(f[0],"ACK") && n==3 &&
               !strcmp(f[1],"DONE") && !strcmp(f[2],"2")) {
        v->area_confirmed=0;
    } else if (!strcmp(f[0],"IDLE") && n==1) {
        v->phase=VISION_IDLE;
        stop_vehicle(v);
    }
}

void Vision_Init(VisionContext *v, VisionTx tx, VisionStop stop, void *user) {
    if (!v) return;
    memset(v,0,sizeof(*v));
    v->tx=tx;
    v->stop=stop;
    v->user=user;
    v->phase=VISION_IDLE;
}

void Vision_RxByte(VisionContext *v, uint8_t b, uint32_t now_ms) {
    char *body;
    if (!v) return;
    if (b=='@') {
        v->rx[0]='@';
        v->rx_len=1;
    } else if (b=='\n') {
        if (v->rx_len) {
            v->rx[v->rx_len]=0;
            body=decode(v->rx);
            if (body) {
                ++v->frames_ok;
                handle(v,body,now_ms);
            } else ++v->frames_bad;
        }
        v->rx_len=0;
    } else if (b=='\r') {
        /* ignore CR */
    } else if (v->rx_len) {
        if (b<32 || b>126 || v->rx_len >= VISION_FRAME_MAX-1) {
            v->rx_len=0;
            ++v->frames_bad;
        } else v->rx[v->rx_len++]=(char)b;
    }
}

int Vision_StartJobTol(VisionContext *v, unsigned code,
                       unsigned tol_x, unsigned tol_y, uint32_t now_ms) {
    char body[48];
    int written, result;
    if (!v || !v->ready || v->phase!=VISION_IDLE ||
        v->pick_ready || v->pick_ack_pending || !job_code_valid(code) ||
        tol_x<1 || tol_x>100 || tol_y<1 || tol_y>100) return 0;
    written=snprintf(body,sizeof(body),"JOB,%u,%u,%u",code,tol_x,tol_y);
    if (written<=0 || written>=(int)sizeof(body)) return 0;
    v->phase=VISION_WAIT_ACK;
    v->job_code=(uint8_t)code;
    v->last_request_ms=now_ms;
    v->result=VISION_RESULT_NONE;
    v->pos.valid=0;
    v->last_error[0]=0;
    v->wait_reason[0]=0;
    if (code==40) v->area_confirmed=0;
    result=Vision_SendBody(v,body);
    if (!result) v->phase=VISION_IDLE;
    return result;
}

int Vision_StartJob(VisionContext *v, unsigned code, uint32_t now_ms) {
    char body[24];
    int written, result;
    if (!v || !v->ready || v->phase!=VISION_IDLE ||
        v->pick_ready || v->pick_ack_pending || !job_code_valid(code)) return 0;
    written=snprintf(body,sizeof(body),"JOB,%u",code);
    if (written<=0 || written>=(int)sizeof(body)) return 0;
    v->phase=VISION_WAIT_ACK;
    v->job_code=(uint8_t)code;
    v->last_request_ms=now_ms;
    v->result=VISION_RESULT_NONE;
    v->pos.valid=0;
    v->last_error[0]=0;
    v->wait_reason[0]=0;
    if (code==40) v->area_confirmed=0;
    result=Vision_SendBody(v,body);
    if (!result) v->phase=VISION_IDLE;
    return result;
}

int Vision_Cancel(VisionContext *v) {
    if (!v || !v->ready || (v->phase==VISION_IDLE && !v->pick_ready)) return 0;
    if (!Vision_SendBody(v,"CANCEL")) return 0;
    v->phase=VISION_WAIT_IDLE;
    stop_vehicle(v);
    return 1;
}

int Vision_ConfirmPick(VisionContext *v) {
    if (!v || !v->pick_ready || v->pick_ack_pending || v->phase!=VISION_IDLE) return 0;
    if (!Vision_SendBody(v,"PICK_OK")) return 0;
    v->pick_ack_pending=1;
    return 1;
}

int Vision_RejectPick(VisionContext *v) {
    if (!v || !v->pick_ready || v->pick_ack_pending || v->phase!=VISION_IDLE) return 0;
    if (!Vision_SendBody(v,"PICK_FAIL")) return 0;
    v->pick_ack_pending=1;
    return 1;
}

int Vision_DoneGroup(VisionContext *v, unsigned group) {
    char body[16];
    if (!v || v->phase!=VISION_IDLE || group<1 || group>4) return 0;
    snprintf(body,sizeof(body),"DONE,%u",group);
    return Vision_SendBody(v,body);
}

int Vision_ResetArea(VisionContext *v) {
    return (v && (v->phase==VISION_IDLE && !v->pick_ready)) ?
           Vision_SendBody(v,"AREA_RESET") : 0;
}

int Vision_Reset(VisionContext *v) {
    if (!v || !v->ready) return 0;
    if (!Vision_SendBody(v,"RESET")) return 0;
    stop_vehicle(v);
    v->phase=VISION_WAIT_IDLE;
    v->pick_ready=0;
    v->pick_ack_pending=0;
    return 1;
}

int Vision_QueryStatus(VisionContext *v) {
    return v ? Vision_SendBody(v,"STATUS") : 0;
}

const VisionPosition *Vision_GetPosition(const VisionContext *v, uint32_t now_ms) {
    if (!v || v->phase!=VISION_RUNNING || !v->pos.valid ||
        (uint32_t)(now_ms-v->pos.time_ms)>VISION_POS_TIMEOUT_MS) return NULL;
    return &v->pos;
}

void Vision_Poll(VisionContext *v, uint32_t now_ms) {
    if (!v) return;
    if (!v->ready && (uint32_t)(now_ms-v->last_probe_ms)>=1000u) {
        v->last_probe_ms=now_ms;
        Vision_SendBody(v,"STATUS");
    }
    if (v->phase==VISION_WAIT_ACK &&
        (uint32_t)(now_ms-v->last_request_ms)>VISION_ACK_TIMEOUT_MS) {
        /* ACK 丢失也必须通知 OpenMV 取消，避免孤儿任务继续发坐标。 */
        set_text(v->last_error,sizeof(v->last_error),"ACK_TIMEOUT");
        v->result=VISION_RESULT_ERROR;
        v->phase=VISION_WAIT_IDLE;
        stop_vehicle(v);
        Vision_SendBody(v,"CANCEL");
    }
    if (v->pos.valid && (uint32_t)(now_ms-v->pos.time_ms)>VISION_POS_TIMEOUT_MS)
        stop_vehicle(v);
}
