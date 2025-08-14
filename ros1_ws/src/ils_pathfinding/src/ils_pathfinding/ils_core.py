import numpy as np
from PIL import Image
import heapq

class GridMap:
    def __init__(self, grid):
        self.grid = grid
        self.h, self.w = grid.shape
    @classmethod
    def from_png(cls, path):
        arr = np.array(Image.open(path).convert("L"))
        grid = (arr < 128).astype(np.uint8)  # 1 is obstacle
        return cls(grid)
    def valid(self, x, y):
        return 0 <= x < self.w and 0 <= y < self.h and self.grid[y,x]==0
    def n4(self, x, y):
        for dx,dy in ((1,0),(-1,0),(0,1),(0,-1)):
            nx,ny=x+dx,y+dy
            if self.valid(nx,ny):
                yield nx,ny

def bline(a, b):
    x1,y1=a; x2,y2=b
    pts=[]; dx=abs(x2-x1); dy=abs(y2-y1)
    sx=1 if x1<x2 else -1; sy=1 if y1<y2 else -1
    x,y=x1,y1
    if dx>dy:
        e=dx//2
        while x!=x2:
            pts.append((x,y)); e-=dy
            if e<0: y+=sy; e+=dx
            x+=sx
    else:
        e=dy//2
        while y!=y2:
            pts.append((x,y)); e-=dx
            if e<0: x+=sx; e+=dy
            y+=sy
    pts.append((x2,y2)); return pts

def ils_mask(gm, start, goal, width):
    if width <= 0: return None
    allow = np.zeros_like(gm.grid, dtype=np.uint8)
    r = max(1, int(min(gm.w, gm.h)*width))
    for x,y in bline(start, goal):
        for dx in range(-r, r+1):
            for dy in range(-r, r+1):
                if abs(dx)+abs(dy) <= r:
                    nx,ny=x+dx,y+dy
                    if 0<=nx<gm.w and 0<=ny<gm.h and gm.grid[ny,nx]==0:
                        allow[ny,nx]=1
    return allow

def manhattan(a,b): return abs(a[0]-b[0])+abs(a[1]-b[1])

class Node:
    __slots__=("x","y","g","h","p")
    def __init__(self,x,y,g=float("inf"),h=0,p=None):
        self.x,self.y,self.g,self.h,self.p=x,y,g,h,p
    @property
    def f(self): return self.g+self.h
    def __lt__(self,o): return self.f<o.f

def reconstruct(n):
    p=[]; cur=n
    while cur is not None:
        p.append((cur.x,cur.y)); cur=cur.p
    p.reverse(); return p

def astar(gm, start, goal, mask=None):
    sx,sy=start; gx,gy=goal
    s=Node(sx,sy,g=0,h=manhattan(start,goal))
    openq=[(s.f,s)]; seen={(sx,sy):s}; closed=set()
    while openq:
        _,u=heapq.heappop(openq)
        if (u.x,u.y)==(gx,gy): return reconstruct(u)
        if (u.x,u.y) in closed: continue
        closed.add((u.x,u.y))
        for nx,ny in gm.n4(u.x,u.y):
            if mask is not None and mask[ny,nx]==0: 
                continue
            ng=u.g+1
            v=seen.get((nx,ny))
            if v is None or ng<v.g:
                if v is None:
                    v=Node(nx,ny); seen[(nx,ny)]=v
                v.g=ng; v.h=manhattan((nx,ny),(gx,gy)); v.p=u
                heapq.heappush(openq,(v.f,v))
    return []
