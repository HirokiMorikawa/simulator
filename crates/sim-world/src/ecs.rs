//! 汎用ECS(Entity-Component-System)コア。設計: `docs/22-roadmap/03-editor-todo.md`
//! D2「汎用ECSを実装する」。
//!
//! **なぜ今これを実装するか**: 同ファイルには以前「汎用ECS/プラグイン機構の要否を
//! 再評価する」という保留項目があり(着手条件「ドキュメント+スキーマ方式では
//! 表現できない要件が具体的に出てきた時点」は不成立のまま維持、と一度結論している)、
//! それ自体は依然として正しい——D2はその結論を覆すものではなく、別の理由(`World`の
//! `Option<T>`ドメインスロットが増分を重ねるごとに線形に太っていく構造そのものを、
//! 汎用的なストレージ+クエリへ置き換えられるかを実地で検証する設計判断)で着手する。
//! 本増分は**土台とワンドメインの実移行**のみを対象とし、11ドメイン全部の一括移行は
//! 意図的にやらない(移行のたびに`state_hash`決定論・シーンJSON往復・wasm境界の
//! 挙動保存を1つずつ検証する必要があり、一度に全部やると壊れたことにすら気付けない)。
//!
//! # 何を提供するか
//! - [`Entity`]: 世代付きindex(`sim_core::BodyId`と同じ形——`index: u32` +
//!   `generation: u32`)。`spawn`/`despawn`で管理する。削除済みスロットの古い
//!   `Entity`ハンドルは、スロットが再利用された後も**新しい占有者を指さない**
//!   (世代が一致しないため`get`/`get_mut`は`None`を返す——世代付きindexの存在理由
//!   そのもの、`tests`モジュールの`despawn_then_respawn_does_not_alias_old_handle`
//!   参照)。
//! - 型ごとのコンポーネントストレージ([`EcsWorld::insert`]/[`remove`]/[`get`]/
//!   [`get_mut`])。内部は`TypeId`をキーにした`HashMap<TypeId, Box<dyn Column>>`で、
//!   各列は`entity.index`で引く`Vec<Option<T>>`(疎だが単純、密なアーキタイプ構造は
//!   採らない——タスクの指示どおり「AAAゲームエンジンではない」ため)。
//! - クエリ([`EcsWorld::iter`]/[`iter_mut`]/[`query2`])。**常に`entity.index`昇順**
//!   でイテレートする——`HashMap`の反復順に依存すると、決定論(`state_hash`が
//!   関わる将来の移行先で特に致命的)を壊すため、列のストレージ自体を`index`で
//!   順序づけられた`Vec`にしてある。
//! - [`Schedule`]: システム(`FnMut(&mut EcsWorld, &mut Ctx)`)を登録順に走らせる
//!   だけの薄いランナー。並列スケジューラは要らない規模(タスクの指示どおり)。
//!
//! # 今回`World`へ実際に繋いだもの・繋いでいないもの
//! `probes`(`Probe`/`ProbeTarget`、`World::add_probe`等)を本増分でECSへ移行した
//! (`lib.rs`の`probes_ecs`フィールド、選定理由は`lib.rs`のdoc参照)。**それ以外の
//! 10ドメイン(mechanics・thermal・em・astro・circuit・gas・conduction_rod・sph・
//! grid_fluid・grid_fluid_3d・soft_body・quantum_1d/2d・brownian・kinetic_gas・
//! ising・fdtd)は一切変更していない**——`Option<T>`スロットのまま。次に移行する
//! なら、剛体運動状態(`sim_mechanics::RigidBodySet`)が最有力候補だが、あちらは
//! 衝突検出・ジョイント・Coupling・wasm数値アクセサ(`body_position_at_f32`等)が
//! 生indexで密結合しているため、`sim-mechanics`crate内部まで踏み込む大改修になる
//! (本増分の対象外、タスクの指示どおり)。各流体/量子/統計ドメインの状態も同様に
//! コンポーネント化できるが、本増分では対象外。

use std::any::{Any, TypeId};
use std::collections::HashMap;

/// 世代付きエンティティindex。`sim_core::BodyId`と同型(`index`+`generation`)。
///
/// 世代不一致の古いハンドルは、スロットが再利用された後も新しい占有者を指さない
/// ——これが世代付きindexを使う唯一の理由なので、`tests`モジュールで明示的に検証する。
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct Entity {
    pub index: u32,
    pub generation: u32,
}

/// 型消去されたコンポーネント列が実装する内部トレイト。`EcsWorld`が
/// `HashMap<TypeId, Box<dyn Column>>`として持つための最小限の共通操作
/// (`despawn`時のスロットクリア、`Clone`)のみを要求する。
trait Column: Any {
    /// `index`番目のスロットを空にする(`despawn`が全列へブロードキャストする)。
    fn clear_slot(&mut self, index: u32);
    fn as_any(&self) -> &dyn Any;
    fn as_any_mut(&mut self) -> &mut dyn Any;
    fn clone_box(&self) -> Box<dyn Column>;
}

/// 具体型`T`の列本体。`entity.index`で引く疎な`Vec<Option<T>>`
/// (モジュールdoc「何を提供するか」参照——密なアーキタイプ構造ではなく、
/// 単純さと`index`昇順反復のしやすさを優先した)。
struct Storage<T> {
    data: Vec<Option<T>>,
}

impl<T> Storage<T> {
    fn new() -> Self {
        Storage { data: Vec::new() }
    }

    fn ensure_len(&mut self, len: usize) {
        if self.data.len() < len {
            self.data.resize_with(len, || None);
        }
    }
}

impl<T: Clone + 'static> Column for Storage<T> {
    fn clear_slot(&mut self, index: u32) {
        if let Some(slot) = self.data.get_mut(index as usize) {
            *slot = None;
        }
    }

    fn as_any(&self) -> &dyn Any {
        self
    }

    fn as_any_mut(&mut self) -> &mut dyn Any {
        self
    }

    fn clone_box(&self) -> Box<dyn Column> {
        Box::new(Storage {
            data: self.data.clone(),
        })
    }
}

/// 汎用ECSワールド。エンティティの生成・削除とコンポーネントの型別ストレージを持つ。
/// `sim_world::World`本体とは別の入れ物(`World`は必要に応じて`EcsWorld`を
/// フィールドとして抱える、`lib.rs`の`probes_ecs`参照)。
pub struct EcsWorld {
    /// スロットごとの現在の世代。`despawn`で+1する。
    generations: Vec<u32>,
    /// スロットが現在生きているか(`generations`と対)。
    alive: Vec<bool>,
    /// 再利用可能な(=`despawn`済みの)スロットindex。
    free_list: Vec<u32>,
    components: HashMap<TypeId, Box<dyn Column>>,
}

impl Clone for EcsWorld {
    fn clone(&self) -> Self {
        EcsWorld {
            generations: self.generations.clone(),
            alive: self.alive.clone(),
            free_list: self.free_list.clone(),
            components: self
                .components
                .iter()
                .map(|(ty, col)| (*ty, col.clone_box()))
                .collect(),
        }
    }
}

impl Default for EcsWorld {
    fn default() -> Self {
        Self::new()
    }
}

impl EcsWorld {
    pub fn new() -> Self {
        EcsWorld {
            generations: Vec::new(),
            alive: Vec::new(),
            free_list: Vec::new(),
            components: HashMap::new(),
        }
    }

    /// 新しいエンティティを生成する。`free_list`にスロットがあればそれを再利用する
    /// (世代は`despawn`時に既に進めてある)、無ければ末尾に新規スロットを増設する。
    pub fn spawn(&mut self) -> Entity {
        if let Some(index) = self.free_list.pop() {
            self.alive[index as usize] = true;
            Entity {
                index,
                generation: self.generations[index as usize],
            }
        } else {
            let index = self.generations.len() as u32;
            self.generations.push(0);
            self.alive.push(true);
            Entity {
                index,
                generation: 0,
            }
        }
    }

    /// エンティティを削除する。全コンポーネント列から即座に値を取り除き
    /// (メモリを無限に溜め込まない)、世代を進めてスロットを再利用可能にする。
    /// 既に削除済み・世代不一致の`entity`に対しては`false`を返し何もしない
    /// (`World`の他の削除APIと同じ「無効な入力は無言で無視する」方針)。
    pub fn despawn(&mut self, entity: Entity) -> bool {
        if !self.is_alive(entity) {
            return false;
        }
        let idx = entity.index as usize;
        self.alive[idx] = false;
        self.generations[idx] = self.generations[idx].wrapping_add(1);
        for column in self.components.values_mut() {
            column.clear_slot(entity.index);
        }
        self.free_list.push(entity.index);
        true
    }

    /// `entity`が現在生存している(削除されておらず、世代が一致する)か。
    pub fn is_alive(&self, entity: Entity) -> bool {
        self.alive
            .get(entity.index as usize)
            .copied()
            .unwrap_or(false)
            && self.generations[entity.index as usize] == entity.generation
    }

    /// 現在生存しているエンティティ数。
    pub fn entity_count(&self) -> usize {
        self.alive.iter().filter(|a| **a).count()
    }

    fn storage<T: Clone + 'static>(&self) -> Option<&Storage<T>> {
        self.components
            .get(&TypeId::of::<T>())
            .map(|c| c.as_any().downcast_ref::<Storage<T>>().expect("型一致"))
    }

    fn storage_mut<T: Clone + 'static>(&mut self) -> &mut Storage<T> {
        self.components
            .entry(TypeId::of::<T>())
            .or_insert_with(|| Box::new(Storage::<T>::new()))
            .as_any_mut()
            .downcast_mut::<Storage<T>>()
            .expect("型一致")
    }

    /// `entity`に型`T`のコンポーネントを設定する。既存の値があれば置き換えて返す。
    /// `entity`が生存していなければ挿入せず`None`を返す(無効な入力は無視する方針)。
    pub fn insert<T: Clone + 'static>(&mut self, entity: Entity, value: T) -> Option<T> {
        if !self.is_alive(entity) {
            return None;
        }
        let storage = self.storage_mut::<T>();
        storage.ensure_len(entity.index as usize + 1);
        storage.data[entity.index as usize].replace(value)
    }

    /// `entity`から型`T`のコンポーネントを取り除き、あれば返す。
    pub fn remove<T: Clone + 'static>(&mut self, entity: Entity) -> Option<T> {
        if !self.is_alive(entity) {
            return None;
        }
        let storage = self.components.get_mut(&TypeId::of::<T>())?;
        let storage = storage
            .as_any_mut()
            .downcast_mut::<Storage<T>>()
            .expect("型一致");
        storage.data.get_mut(entity.index as usize)?.take()
    }

    pub fn get<T: Clone + 'static>(&self, entity: Entity) -> Option<&T> {
        if !self.is_alive(entity) {
            return None;
        }
        self.storage::<T>()?
            .data
            .get(entity.index as usize)?
            .as_ref()
    }

    pub fn get_mut<T: Clone + 'static>(&mut self, entity: Entity) -> Option<&mut T> {
        if !self.is_alive(entity) {
            return None;
        }
        self.components
            .get_mut(&TypeId::of::<T>())?
            .as_any_mut()
            .downcast_mut::<Storage<T>>()
            .expect("型一致")
            .data
            .get_mut(entity.index as usize)?
            .as_mut()
    }

    /// 型`T`のコンポーネントを持つ全エンティティを`index`昇順で反復する
    /// (モジュールdoc「何を提供するか」参照、決定論のため反復順を固定している)。
    pub fn iter<T: Clone + 'static>(&self) -> impl Iterator<Item = (Entity, &T)> + '_ {
        let generations = &self.generations;
        let alive = &self.alive;
        self.storage::<T>()
            .into_iter()
            .flat_map(|s| s.data.iter().enumerate())
            .filter_map(move |(idx, slot)| {
                let value = slot.as_ref()?;
                if !*alive.get(idx)? {
                    return None;
                }
                Some((
                    Entity {
                        index: idx as u32,
                        generation: generations[idx],
                    },
                    value,
                ))
            })
    }

    /// [`iter`]の可変版。
    pub fn iter_mut<T: Clone + 'static>(&mut self) -> impl Iterator<Item = (Entity, &mut T)> + '_ {
        let generations = &self.generations;
        let alive = &self.alive;
        self.components
            .get_mut(&TypeId::of::<T>())
            .and_then(|c| c.as_any_mut().downcast_mut::<Storage<T>>())
            .into_iter()
            .flat_map(|s| s.data.iter_mut().enumerate())
            .filter_map(move |(idx, slot)| {
                let value = slot.as_mut()?;
                if !*alive.get(idx)? {
                    return None;
                }
                Some((
                    Entity {
                        index: idx as u32,
                        generation: generations[idx],
                    },
                    value,
                ))
            })
    }

    /// 型`A`・`B`の両方を持つ全エンティティを`index`昇順で反復する
    /// (`A`の列を軸に`B`を都度引く単純な実装——タスクの指示どおりcache最適化された
    /// アーキタイプ結合ではない、この規模では十分)。
    pub fn query2<A: Clone + 'static, B: Clone + 'static>(
        &self,
    ) -> impl Iterator<Item = (Entity, &A, &B)> + '_ {
        self.iter::<A>()
            .filter_map(move |(e, a)| self.get::<B>(e).map(|b| (e, a, b)))
    }
}

/// 登録順にシステムを走らせるだけの薄いスケジューラ。システムは
/// `FnMut(&mut EcsWorld, &mut Ctx)`——`Ctx`は各システムが必要とする外部文脈
/// (例: 物理ドメインからサンプルした値の受け渡し。`EcsWorld`自体が知らない情報を
/// 系に渡すための穴、`lib.rs`の`probes_ecs`移行で使う`Vec<f64>`参照)。
/// `Ctx = ()`ならシステムは`EcsWorld`だけで完結する。
///
/// 並列実行やシステム間の依存グラフ解決はしない(タスクの指示どおり、この規模の
/// 物理シムには過剰)。
type SystemFn<Ctx> = Box<dyn FnMut(&mut EcsWorld, &mut Ctx)>;

pub struct Schedule<Ctx = ()> {
    systems: Vec<SystemFn<Ctx>>,
}

impl<Ctx> Default for Schedule<Ctx> {
    fn default() -> Self {
        Schedule {
            systems: Vec::new(),
        }
    }
}

impl<Ctx> Schedule<Ctx> {
    pub fn new() -> Self {
        Self::default()
    }

    /// システムを末尾へ登録する。
    pub fn add_system(
        &mut self,
        system: impl FnMut(&mut EcsWorld, &mut Ctx) + 'static,
    ) -> &mut Self {
        self.systems.push(Box::new(system));
        self
    }

    /// 登録順に全システムを1回ずつ走らせる。
    pub fn run(&mut self, world: &mut EcsWorld, ctx: &mut Ctx) {
        for system in &mut self.systems {
            system(world, ctx);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[derive(Clone, Debug, PartialEq)]
    struct Position(f64, f64);
    #[derive(Clone, Debug, PartialEq)]
    struct Velocity(f64, f64);
    #[derive(Clone, Debug, PartialEq)]
    struct Health(i32);

    #[test]
    fn spawn_assigns_increasing_index_and_generation_zero() {
        let mut world = EcsWorld::new();
        let a = world.spawn();
        let b = world.spawn();
        assert_eq!(
            a,
            Entity {
                index: 0,
                generation: 0
            }
        );
        assert_eq!(
            b,
            Entity {
                index: 1,
                generation: 0
            }
        );
        assert_eq!(world.entity_count(), 2);
    }

    #[test]
    fn despawn_then_respawn_does_not_alias_old_handle() {
        // 世代付きindexの存在理由そのもの: 削除済みスロットを指す古いハンドルは、
        // スロットが再利用された後も新しい占有者へアクセスできてはならない。
        let mut world = EcsWorld::new();
        let a = world.spawn();
        world.insert(a, Position(1.0, 2.0));
        assert!(world.despawn(a));
        assert_eq!(world.entity_count(), 0);

        let b = world.spawn();
        // スロットindexは再利用されるが世代は進んでいる。
        assert_eq!(b.index, a.index);
        assert_ne!(b.generation, a.generation);

        world.insert(b, Position(9.0, 9.0));

        // 古いハンドル`a`は新しい占有者`b`を指さない。
        assert_eq!(world.get::<Position>(a), None);
        assert!(!world.is_alive(a));
        assert_eq!(world.get::<Position>(b), Some(&Position(9.0, 9.0)));

        // 二重despawnは無視される(falseを返すだけでパニックしない)。
        assert!(!world.despawn(a));
    }

    #[test]
    fn despawn_clears_all_component_types_for_the_slot() {
        let mut world = EcsWorld::new();
        let a = world.spawn();
        world.insert(a, Position(1.0, 1.0));
        world.insert(a, Velocity(2.0, 2.0));
        world.despawn(a);

        let b = world.spawn();
        assert_eq!(b.index, a.index);
        // Position は上書きされたが、Velocity は一度も入れていないので None のまま。
        assert_eq!(world.get::<Position>(b), None);
        assert_eq!(world.get::<Velocity>(b), None);
    }

    #[test]
    fn insert_get_get_mut_remove_roundtrip() {
        let mut world = EcsWorld::new();
        let e = world.spawn();
        assert_eq!(world.get::<Position>(e), None);

        let old = world.insert(e, Position(1.0, 2.0));
        assert_eq!(old, None);
        assert_eq!(world.get::<Position>(e), Some(&Position(1.0, 2.0)));

        if let Some(p) = world.get_mut::<Position>(e) {
            p.0 = 5.0;
        }
        assert_eq!(world.get::<Position>(e), Some(&Position(5.0, 2.0)));

        let replaced = world.insert(e, Position(9.0, 9.0));
        assert_eq!(replaced, Some(Position(5.0, 2.0)));

        let removed = world.remove::<Position>(e);
        assert_eq!(removed, Some(Position(9.0, 9.0)));
        assert_eq!(world.get::<Position>(e), None);
        // 二重removeはNoneを返すだけ。
        assert_eq!(world.remove::<Position>(e), None);
    }

    #[test]
    fn insert_on_dead_entity_is_a_noop() {
        let mut world = EcsWorld::new();
        let e = world.spawn();
        world.despawn(e);
        assert_eq!(world.insert(e, Position(1.0, 1.0)), None);
        assert_eq!(world.get::<Position>(e), None);
    }

    #[test]
    fn iter_returns_only_entities_with_the_component_in_index_order() {
        let mut world = EcsWorld::new();
        let a = world.spawn();
        let b = world.spawn();
        let c = world.spawn();
        world.insert(a, Position(1.0, 0.0));
        world.insert(c, Position(3.0, 0.0));
        // b にはPositionを入れない。

        let collected: Vec<(Entity, Position)> = world
            .iter::<Position>()
            .map(|(e, p)| (e, p.clone()))
            .collect();
        assert_eq!(
            collected,
            vec![(a, Position(1.0, 0.0)), (c, Position(3.0, 0.0))]
        );
        let _ = b;
    }

    #[test]
    fn query2_returns_intersection_of_two_component_types() {
        let mut world = EcsWorld::new();
        let a = world.spawn();
        let b = world.spawn();
        let c = world.spawn();
        world.insert(a, Position(1.0, 0.0));
        world.insert(a, Velocity(0.1, 0.0));
        world.insert(b, Position(2.0, 0.0));
        // b には Velocity が無い。
        world.insert(c, Velocity(0.3, 0.0));
        // c には Position が無い。

        let matched: Vec<Entity> = world
            .query2::<Position, Velocity>()
            .map(|(e, _, _)| e)
            .collect();
        assert_eq!(matched, vec![a]);
    }

    #[test]
    fn system_runs_and_mutates_state_via_a_query() {
        // 「システムがクエリ経由で状態を変更する」ことの最小デモ:
        // Position を Velocity ぶんだけ毎フレーム進める、を Schedule 経由で1回走らせる。
        let mut world = EcsWorld::new();
        let a = world.spawn();
        world.insert(a, Position(0.0, 0.0));
        world.insert(a, Velocity(1.0, 2.0));
        let b = world.spawn();
        world.insert(b, Position(10.0, 10.0));
        // b は Velocity を持たないので動かないはず。

        let mut schedule: Schedule<()> = Schedule::new();
        schedule.add_system(|w: &mut EcsWorld, _ctx: &mut ()| {
            let deltas: Vec<(Entity, f64, f64)> = w
                .query2::<Position, Velocity>()
                .map(|(e, p, v)| (e, p.0 + v.0, p.1 + v.1))
                .collect();
            for (e, x, y) in deltas {
                if let Some(p) = w.get_mut::<Position>(e) {
                    *p = Position(x, y);
                }
            }
        });
        schedule.run(&mut world, &mut ());

        assert_eq!(world.get::<Position>(a), Some(&Position(1.0, 2.0)));
        assert_eq!(world.get::<Position>(b), Some(&Position(10.0, 10.0)));
    }

    #[test]
    fn schedule_runs_systems_in_registration_order() {
        let mut world = EcsWorld::new();
        let e = world.spawn();
        world.insert(e, Health(0));

        let mut schedule: Schedule<()> = Schedule::new();
        schedule.add_system(|w: &mut EcsWorld, _: &mut ()| {
            if let Some(h) = w.get_mut::<Health>(Entity {
                index: 0,
                generation: 0,
            }) {
                h.0 += 10;
            }
        });
        schedule.add_system(|w: &mut EcsWorld, _: &mut ()| {
            if let Some(h) = w.get_mut::<Health>(Entity {
                index: 0,
                generation: 0,
            }) {
                h.0 *= 2;
            }
        });
        schedule.run(&mut world, &mut ());

        // (0 + 10) * 2 = 20。順序が入れ替わっていれば (0 * 2) + 10 = 10 になるはず。
        assert_eq!(world.get::<Health>(e), Some(&Health(20)));
    }

    #[test]
    fn clone_preserves_entities_and_components_independently() {
        let mut world = EcsWorld::new();
        let e = world.spawn();
        world.insert(e, Position(1.0, 1.0));

        let mut cloned = world.clone();
        if let Some(p) = cloned.get_mut::<Position>(e) {
            p.0 = 99.0;
        }

        assert_eq!(world.get::<Position>(e), Some(&Position(1.0, 1.0)));
        assert_eq!(cloned.get::<Position>(e), Some(&Position(99.0, 1.0)));
    }
}
