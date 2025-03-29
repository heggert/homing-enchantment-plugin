package art.chibi.homingenchant;

import net.kyori.adventure.text.Component;
import org.bukkit.NamespacedKey;
import org.bukkit.enchantments.Enchantment;
import org.bukkit.enchantments.EnchantmentTarget;
import org.bukkit.entity.EntityCategory;
import org.bukkit.entity.EntityType;
import org.bukkit.inventory.EquipmentSlotGroup;
import org.bukkit.inventory.ItemStack;
import org.bukkit.inventory.ItemType;

import io.papermc.paper.enchantments.EnchantmentRarity;
import io.papermc.paper.registry.RegistryKey;
import io.papermc.paper.registry.TypedKey;
import io.papermc.paper.registry.set.RegistrySet;
import io.papermc.paper.registry.set.RegistryKeySet;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

import java.util.Set;

public class HomingEnchantment extends Enchantment {

    public static final NamespacedKey KEY = NamespacedKey.fromString("homing");

    public HomingEnchantment() {
        super();
    }

    @Override
    public @NotNull NamespacedKey getKey() {
        return new NamespacedKey("homingenchant", "homing");
    }

    @Override
    public @NotNull String getTranslationKey() {
        return "enchantment.homingenchant.homing";
    }

    @Override
    public @NotNull String getName() {
        return "Homing";
    }

    @Override
    public int getMaxLevel() {
        return 1;
    }

    @Override
    public int getStartLevel() {
        return 1;
    }

    @Override
    public @NotNull EnchantmentTarget getItemTarget() {
        return EnchantmentTarget.BOW;
    }

    @Override
    public boolean isTreasure() {
        return false;
    }

    @Override
    public boolean isCursed() {
        return false;
    }

    @Override
    public boolean conflictsWith(@NotNull Enchantment other) {
        return false;
    }

    @Override
    public boolean canEnchantItem(@NotNull ItemStack item) {
        return true;
    }

    @Override
    public @NotNull Component displayName(int level) {
        return Component.text("Homing");
    }

    @Override
    public boolean isTradeable() {
        return true;
    }

    @Override
    public boolean isDiscoverable() {
        return false;
    }

    @Override
    public int getMinModifiedCost(int level) {
        return 1;
    }

    @Override
    public int getMaxModifiedCost(int level) {
        return 40;
    }

    @Override
    public int getAnvilCost() {
        return 40;
    }

    @SuppressWarnings("removal")
    @Override
    public @NotNull EnchantmentRarity getRarity() {
        return EnchantmentRarity.RARE;
    }

    @Override
    public float getDamageIncrease(int level, @NotNull EntityCategory entityCategory) {
        return 0;
    }

    @Override
    public float getDamageIncrease(int level, @NotNull EntityType entityType) {
        return 0;
    }

    @Override
    public @NotNull Set<EquipmentSlotGroup> getActiveSlotGroups() {
        return Set.of(EquipmentSlotGroup.MAINHAND);
    }

    @Override
    public @NotNull Component description() {
        return Component.text("Arrows will home in on the nearest entity");
    }

    @Override
    public @NotNull RegistryKeySet<ItemType> getSupportedItems() {
        return RegistrySet.keySet(RegistryKey.ITEM,
                TypedKey.create(RegistryKey.ITEM, ItemType.BOW.getKey()),
                TypedKey.create(RegistryKey.ITEM, ItemType.CROSSBOW.getKey()));
    }

    @Override
    public @Nullable RegistryKeySet<ItemType> getPrimaryItems() {
        return RegistrySet.keySet(RegistryKey.ITEM,
                TypedKey.create(RegistryKey.ITEM, ItemType.BOW.getKey()));
    }

    @Override
    public int getWeight() {
        return 1;
    }

    @Override
    public @NotNull RegistryKeySet<Enchantment> getExclusiveWith() {
        return RegistrySet.keySet(RegistryKey.ENCHANTMENT,
                TypedKey.create(RegistryKey.ENCHANTMENT, Enchantment.DENSITY.getKey()));
    }

    @Override
    public @NotNull String translationKey() {
        return "enchantment.homingenchant.homing";
    }
}
